package gay.zharel.fastlane

import dev.nextftc.control.geometry.*
import dev.nextftc.units.Inches
import dev.nextftc.units.Measure
import dev.nextftc.units.measuretypes.Angle
import dev.nextftc.units.measuretypes.Distance
import dev.nextftc.units.measuretypes.Voltage
import dev.nextftc.units.unittypes.DistanceUnit
import dev.nextftc.units.unittypes.PerUnit
import dev.nextftc.units.unittypes.TimeUnit
import java.util.PriorityQueue
import java.util.Queue
import java.util.function.Consumer
import java.util.function.Function

data class Waypoint(val pos: Vector2d<DistanceUnit>) {
    val pose get() = Pose2d(this.pos, Rotation2d.zero)
}

val Pose2d.waypoint get() = Waypoint(this.position)

data class RotationTarget(val t: Double, val angle: Rotation2d)

data class ConstraintZone(val startT: Double, val endT: Double, val topSpeed: Voltage)

data class FastlanePath(
    val waypoints: List<Waypoint>,
    val rotationTargets: List<RotationTarget>,
    val constraintZones: List<ConstraintZone>,
    val name: String
)
interface Localizer {
    fun getPose(): Pose2d
    fun getVelocity(): PoseVelocity2d
    fun update()
}

interface Controller<X, U> {
    fun get(pv: X, sp: X): U
}

interface KinematicsPropogator {
    fun getProjectedPose(pos: Pose2d, vel: PoseVelocity2d): Pose2d
}

class MecanumKinematicsPropogator(
    val axialCoast: Function<Measure<PerUnit<DistanceUnit, TimeUnit>>, Distance>,
    val coaxialCoast: Function<Measure<PerUnit<DistanceUnit, TimeUnit>>, Distance>
) : KinematicsPropogator {
    override fun getProjectedPose(
        pos: Pose2d, vel: PoseVelocity2d
    ): Pose2d {
        // convert vel to robot relative
        // chassis vel helper does this for us. thanks sensible geometry classes!
        val rcVel = vel.toChassis(pos.heading)

        // propogate independently
        val rcTwist =
            Twist2d(axialCoast.apply(rcVel.linearVel.x), coaxialCoast.apply(rcVel.linearVel.y), Rotation2d.zero)
        return pos.plus(rcTwist)
    }

}

/**
 * @param driveFunction consumes a PoseVoltage2d to drive the robot
 * to drive the dt that way
 */
class Fastlane(
    val localizer: Localizer,
    val kinematicsPropogator: KinematicsPropogator,
    val headingController: Controller<Double, Voltage>,
    val drivetrainController: Controller<Distance, Voltage>,
    val driveFunction: Consumer<PoseVoltage2d>,
    val tolerance: Distance
) {

    // EXTERNAL PARAMETERS
    var points: List<Waypoint> = emptyList()
        set(value) {
            field = value
            distanceToEnd = Inches.of(
                value
                    .foldIndexed(0.0) { i, acc, pose ->
                        if (i > 1) acc + value[i - 1].pose.distanceTo(pose.pose) else 0.0
                    }
            )
        }

    var rotationTargets: List<RotationTarget> = emptyList()
    var constraintZones: List<ConstraintZone> = emptyList()
    var pathName: String = ""

    fun setPath(path: FastlanePath) {
        this.points = path.waypoints
        this.rotationTargets = path.rotationTargets
        this.constraintZones = path.constraintZones
        this.pathName = path.name
        reset()
    }

    fun fromPathplannerPath(name: String) {
        val cached = cachedPaths[name]
        if (cached != null) {
            setPath(cached)
        } else {
            cachedPaths[name] = PathPlannerParser.parse(PathplannerFileManager.getPathFile(name)).fastlanePath
            setPath(cachedPaths[name] ?: throw Error("wtf twin like how"))
        }
    }

    val cachedPaths: MutableMap<String, FastlanePath> = mutableMapOf()

    /**
     * loads paths before the auto starts or something
     */
    fun loadPathplannerPaths(vararg names: String) {
        names.forEach {
            cachedPaths[it] = PathPlannerParser.parse(PathplannerFileManager.getPathFile(it)).fastlanePath
        }
    }

    // INTERNAL STATE TRACKING
    private var index: Int = 1
    private lateinit var distanceToEnd: Distance

    private val currentEndPoint get() = points[index]
    private val currentStartPoint get() = points[index - 1]
    private val lastPoint get() = points.last()

    private var rotationIndex: Int = 0
    private val rotationTarget: RotationTarget get() {
        while (rotationTargets[rotationIndex].t < currentT) {
            rotationIndex++
        }
        return rotationTargets[rotationIndex]
    }

    private var constraintIndex: Int = 0
    private val constraintZone: ConstraintZone get() {
        while (constraintZones[constraintIndex].endT < currentT) {
            constraintIndex++
        }
        return constraintZones[constraintIndex]
    }


    // for usage with triggers and such
    var lastT = 0.0
    var currentT = 0.0
    var isFinished = false

    /**
     * Resets all internal states. Call before any follower run.
     */
    fun reset() {
        index = 0
        constraintIndex = 0
        rotationIndex = 0
        distanceToEnd = Inches.of(
            points
                .foldIndexed(0.0) { i, acc, pose ->
                    if (i > 1) acc + points[i - 1].pose.distanceTo(pose.pose) else 0.0
                }
        )
        currentT = 0.0
        isFinished = false
    }

    fun get(): PoseVoltage2d {
        val pose = localizer.getPose()
        val vel = localizer.getVelocity()

        val propogatedPose = kinematicsPropogator.getProjectedPose(pose, vel)

        // end condition check
        isFinished = Inches.of(propogatedPose.distanceTo(lastPoint.pose)) < tolerance
                && index == points.size - 1

        // while projected pose is past point, move on!
        while (index != points.size - 1
            && propogatedPose.closestParameterOnSegment(currentStartPoint.pose, currentEndPoint.pose) == 1.0
        ) {
            index++
            // subtract off this segment
            if (index != points.size - 1) {
                distanceToEnd -= Inches.of(currentStartPoint.pose.distanceTo(currentEndPoint.pose))
            }
        }

        // irrelevant amount of code dupe
        lastT = currentT
        currentT = propogatedPose.closestParameterOnSegment(currentStartPoint.pose, currentEndPoint.pose) + index - 1

        // translation!!
        // get the drive controller output, always wrt distance remaining to last. units ftc power!
        val controlMagnitude = drivetrainController.get(
            distanceToEnd + Inches.of(propogatedPose.distanceTo(currentEndPoint.pose)),
            Inches.of(0.0)
        ).coerceIn(-constraintZone.topSpeed, constraintZone.topSpeed)

        var controlDirection = (currentEndPoint.pose - propogatedPose).line
        controlDirection /= controlDirection.norm()
        val controlDirectionVolts = Vector2d(
            controlMagnitude * controlDirection.x.baseUnitMagnitude,
            controlMagnitude * controlDirection.y.baseUnitMagnitude
        )

        // heading
        val headingImpulse = headingController.get(pose.heading.toDouble(), rotationTarget.angle.toDouble())

        return PoseVoltage2d(controlDirectionVolts, headingImpulse)
    }
}
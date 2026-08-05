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
    val name: String,
    val state: FastlanePathState = FastlanePathState()
) {
    val rotationTarget: RotationTarget get() = rotationTargets[state.rotationIndex]
    val constraintZone: ConstraintZone get() = constraintZones[state.constraintIndex]

    val currentStartWaypoint: Waypoint get() = waypoints[state.waypointIndex - 1]
    val currentEndWaypoint: Waypoint get() = waypoints[state.waypointIndex]

    fun updateState(t: Double) {
        while (t > rotationTarget.t) {
            state.rotationIndex++;
        }
        while (t > constraintZone.endT) {
            state.constraintIndex++;
        }
    }

    fun getMaxVoltage(t: Double): Voltage {
        return if (t > constraintZone.startT && t < constraintZone.endT) {
            constraintZone.topSpeed
        } else {
            Throttle.of(1.0);
        }
    }

}

data class FastlanePathState(
    var waypointIndex: Int = 1,
    var rotationIndex: Int = 0,
    var constraintIndex: Int = 0,
) {
    fun reset() {
        this.constraintIndex = 0
        this.rotationIndex = 0
        this.waypointIndex = 1
    }
}
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

object FastlanePaths {
    val cachedPaths: MutableMap<String, FastlanePath> = mutableMapOf()

    /**
     * loads paths before the auto starts or something
     */
    fun loadPathplannerPaths(vararg names: String) {
        names.forEach {
            cachedPaths[it] = PathPlannerParser.parse(PathplannerFileManager.getPathFile(it)).fastlanePath
        }
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

    var path: FastlanePath = FastlanePath(emptyList(), emptyList(), emptyList(), "")
        set(value) {
            field = value
            reset()
        }
    val points: List<Waypoint> get() = path.waypoints
    val rotationTargets: List<RotationTarget> get() = path.rotationTargets
    val constraintZones: List<ConstraintZone> get() = path.constraintZones
    val pathName: String get() = path.name

    // INTERNAL STATE TRACKING
    private lateinit var distanceToEnd: Distance
    private var index by path.state::waypointIndex
    private val lastPoint get() = points.last()

    // for usage with triggers and such
    var lastT = 0.0
    var currentT = 0.0
    var isFinished = false

    /**
     * Resets all internal states. Call before any follower run.
     */
    fun reset() {
        path.state.reset()
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
            && propogatedPose.closestParameterOnSegment(path.currentStartWaypoint.pose, path.currentEndWaypoint.pose) == 1.0
        ) {
            index++
            // subtract off this segment
            if (index != points.size - 1) {
                distanceToEnd -= Inches.of(path.currentStartWaypoint.pose.distanceTo(path.currentEndWaypoint.pose))
            }
        }

        // irrelevant amount of code dupe
        lastT = currentT
        currentT = propogatedPose.closestParameterOnSegment(path.currentStartWaypoint.pose, path.currentEndWaypoint.pose) + index - 1

        // update targets before controllers take over
        path.updateState(currentT)

        // translation!!
        // get the drive controller output, always wrt distance remaining to last. units ftc power!
        val controlMagnitude = drivetrainController.get(
            distanceToEnd + Inches.of(propogatedPose.distanceTo(path.currentEndWaypoint.pose)),
            Inches.of(0.0)
        ).coerceIn(-path.getMaxVoltage(currentT), path.getMaxVoltage(currentT))

        var controlDirection = (path.currentEndWaypoint.pose - propogatedPose).line
        controlDirection /= controlDirection.norm()
        val controlDirectionVolts = Vector2d(
            controlMagnitude * controlDirection.x.baseUnitMagnitude,
            controlMagnitude * controlDirection.y.baseUnitMagnitude
        )

        // heading
        val headingImpulse = headingController.get(pose.heading.toDouble(), path.rotationTarget.angle.toDouble())

        return PoseVoltage2d(controlDirectionVolts, headingImpulse)
    }
}
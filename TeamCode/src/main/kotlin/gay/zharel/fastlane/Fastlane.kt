package gay.zharel.fastlane

import dev.nextftc.control.geometry.Pose2d
import dev.nextftc.control.geometry.PoseVelocity2d
import dev.nextftc.control.geometry.Rotation2d
import dev.nextftc.control.geometry.Twist2d
import dev.nextftc.control.geometry.Vector2d
import dev.nextftc.units.Inches
import dev.nextftc.units.InchesPerSecond
import dev.nextftc.units.Measure
import dev.nextftc.units.Meters
import dev.nextftc.units.measuretypes.Distance
import dev.nextftc.units.measuretypes.LinearVelocity
import dev.nextftc.units.measuretypes.Voltage
import dev.nextftc.units.unittypes.DistanceUnit
import dev.nextftc.units.unittypes.LinearVelocityUnit
import dev.nextftc.units.unittypes.PerUnit
import dev.nextftc.units.unittypes.TimeUnit
import java.util.function.Consumer
import java.util.function.Function

data class Waypoint(val pose: Pose2d, val topThrottle: Voltage = NormalizedThrottle.of(Double.POSITIVE_INFINITY))

fun Pose2d.toWaypoint() = Waypoint(this)
interface Localizer {
    fun getPose(): Pose2d
    fun getVelocity(): PoseVelocity2d
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
        val rcTwist = Twist2d(axialCoast.apply(rcVel.linearVel.x), coaxialCoast.apply(rcVel.linearVel.y), Rotation2d.zero)
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
                    .foldIndexed(0.0) {
                        i, acc, pose ->
                        if (i > 1) acc + value[i - 1].pose.distanceTo(pose.pose) else 0.0
                    }
            )
        }

    // INTERNAL STATE TRACKING
    private var index: Int = 1
    private lateinit var distanceToEnd: Distance

    private val currentEndPoint get() = points[index]
    private val currentStartPoint get() = points[index - 1]
    private val lastPoint get() = points.last()

    // for usage with triggers and such
    var currentT = 0.0
    var isFinished = false

    /**
     * Resets all internal states. Call before any follower run.
     */
    fun reset() {
        index = 0
        distanceToEnd = Inches.of(
            points
                .foldIndexed(0.0) {
                        i, acc, pose ->
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
            && propogatedPose.closestParameterOnSegment(currentStartPoint.pose, currentEndPoint.pose) == 1.0) {
            index++
            // subtract off this segment
            if (index != points.size - 1) {
                distanceToEnd -= Inches.of(currentStartPoint.pose.distanceTo(currentEndPoint.pose))
            }
        }

        // irrelevant amount of code dupe
        currentT = propogatedPose.closestParameterOnSegment(currentStartPoint.pose, currentEndPoint.pose) + index - 1

        // translation!!
        // get the drive controller output, always wrt distance remaining to last. units ftc power!
        val controlMagnitude = drivetrainController.get(
            distanceToEnd + Inches.of(propogatedPose.distanceTo(currentEndPoint.pose)),
            Inches.of(0.0)
        ).coerceIn(-currentEndPoint.topThrottle, currentEndPoint.topThrottle)

        var controlDirection = (currentEndPoint.pose - propogatedPose).line
        controlDirection /= controlDirection.norm()
        val controlDirectionVolts = Vector2d(controlMagnitude * controlDirection.x.baseUnitMagnitude,
            controlMagnitude * controlDirection.y.baseUnitMagnitude)

        // heading
        val headingImpulse = headingController.get(pose.heading.toDouble(), currentEndPoint.pose.heading.toDouble())

        return PoseVoltage2d(controlDirectionVolts, headingImpulse)
    }
}
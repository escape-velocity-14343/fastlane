package gay.zharel.fastlane

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import dev.nextftc.control.geometry.Vector2d
import gay.zharel.fateweaver.flight.FateLogManager
import gay.zharel.fateweaver.log.FateLogWriter
import java.util.function.Consumer

enum class Axis {
    FORWARD,
    STRAFE
}

/**
 * Update this object to represent your mechanisms!
 */
object TuningMechanisms {
    lateinit var localizer: Localizer
    lateinit var driveFunction: Consumer<PoseVoltage2d>
}

@Config
/**
 * Measures the stopping distance of a drivetrain in one axis.
 * This tuner will require a full field of space in one axis!
 */
abstract class StoppingDistanceTuner : LinearOpMode() {

    companion object {
        @JvmStatic
        val axis = Axis.FORWARD
    }

    /**
     * Call after <code>waitForStart()</code> in your opmode.
     */
    fun runTuner() {

        val forwards = PoseVoltage2d(
            if (axis == Axis.FORWARD) Vector2d(
                NormalizedThrottle.of(1.0),
                NormalizedThrottle.of(0.0)
            ) else Vector2d(
                NormalizedThrottle.of(0.0),
                NormalizedThrottle.of(1.0)
            ), NormalizedThrottle.of(0.0)
        )

        val backwards = PoseVoltage2d(forwards.linearVolt * -1.0, forwards.angVolt)

        val writer = FateLogManager.start("StoppingDistanceTest${axis.name}-${System.nanoTime()}")
        val localizer = TuningMechanisms.localizer

        // run 5 data points worth of tests
        val timings = arrayListOf(0.1, 0.2, 0.3, 0.5, 1.0)

        for (timing in timings) {
            for (trial in 1..3) {
                val testName = "Test-${timing}-${trial}"
                writer.write(testName, "Test Start. System time: ${System.nanoTime()}")

                var startTime = System.nanoTime()
                writer.write(testName, "Forward Pass Start. System time: ${System.nanoTime()}")

                TuningMechanisms.driveFunction.accept(forwards)

                while (System.nanoTime() - startTime < timing * 1e9) {
                    localizer.update()
                }

                writer.write(testName, "Forward Pass End. System time: ${System.nanoTime()}")
                var endPose = localizer.getPose()
                writer.write("endVelocity", localizer.getVelocity().linearVel.norm())

                while (localizer.getVelocity().linearVel.norm() > 0.01) {
                    localizer.update()
                }

                writer.write(testName, "Forward Coast End. System time: ${System.nanoTime()}")
                var coastPose = localizer.getPose()
                writer.write("endDistance", coastPose.distanceTo(endPose))

                startTime = System.nanoTime()
                writer.write(testName, "Backward Pass Start. System time: ${System.nanoTime()}")

                TuningMechanisms.driveFunction.accept(backwards)

                while (System.nanoTime() - startTime < timing * 1e9) {
                    localizer.update()
                }

                writer.write(testName, "Backward Pass End. System time: ${System.nanoTime()}")
                endPose = localizer.getPose()
                writer.write("endVelocity", localizer.getVelocity().linearVel.norm())

                while (localizer.getVelocity().linearVel.norm() > 0.01) {
                    localizer.update()
                }

                writer.write(testName, "Backward Coast End. System time: ${System.nanoTime()}")
                coastPose = localizer.getPose()
                writer.write("endDistance", coastPose.distanceTo(endPose))
            }
        }
    }
}
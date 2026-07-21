package gay.zharel.fastlane

import dev.nextftc.units.Volts
import dev.nextftc.units.unittypes.VoltageUnit

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.control.geometry.Rotation2d
import dev.nextftc.control.geometry.Vector2d
import dev.nextftc.units.Measure
import dev.nextftc.units.measuretypes.AngularVelocity
import dev.nextftc.units.unittypes.DistanceUnit
import dev.nextftc.units.unittypes.PerUnit
import dev.nextftc.units.unittypes.TimeUnit

/**
 * Global object for synchronized and IO-efficient voltage reads.
 */
object VoltageCache {

    private lateinit var voltageSensor: VoltageSensor
    private var cachedVoltage: Double = 0.0
    private val timeElapsed: ElapsedTime = ElapsedTime()

    @JvmStatic
    val nominalVoltage: Double = 12.5

    @JvmStatic
    val readIntervalSeconds: Double = 0.5

    val currentVoltage: Double
        get() {
            if (timeElapsed.seconds() > readIntervalSeconds) {
                cachedVoltage = voltageSensor.voltage
                timeElapsed.reset()
            }
            return cachedVoltage
        }

    /**
     * Initializes the VoltageCache with the voltage sensor for the OpMode.
     */
    fun init(hardwareMap: HardwareMap) {
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next()
        cachedVoltage = voltageSensor.voltage
        timeElapsed.reset()
    }

}

val Throttle = VoltageUnit(Volts,
    {throttle: Double -> throttle * VoltageCache.currentVoltage},
    {volt: Double -> volt / VoltageCache.currentVoltage},
    "throttle", "thr")

val NormalizedThrottle = VoltageUnit(Volts, VoltageCache.nominalVoltage,
    "norm-throttle", "thrN")

data class PoseVoltage2d(
    @JvmField val linearVolt: Vector2d<VoltageUnit>,
    @JvmField val angVolt: Measure<VoltageUnit>,
)
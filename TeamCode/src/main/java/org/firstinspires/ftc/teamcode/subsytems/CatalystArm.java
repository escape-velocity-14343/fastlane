package org.firstinspires.ftc.teamcode.subsytems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.button.Trigger;
import dev.nextftc.units.Measure;
import dev.nextftc.units.measuretypes.Angle;
import dev.nextftc.units.measuretypes.Distance;
import dev.nextftc.units.measuretypes.Voltage;
import dev.nextftc.units.unittypes.AngleUnit;
import dev.nextftc.units.unittypes.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.EnderLog;

import static dev.nextftc.units.Units.Degrees;
import static dev.nextftc.units.Units.Inches;
import static gay.zharel.fastlane.UnitsKt.Throttle;

/**
 * does the catalyst intaking and outtaking or sum
 */
@Config
public class CatalystArm extends SubsystemBase {
    // MODE HANDLING (optional)
    // modes determine *how* a subsystem runs. states are descriptions of a subsystem's target state (control theory state).
    // modes are meant for bringup/tuning. states are meant for systems operating at full capacity (mode.FULL).
    public enum CatalystArmMode {
        DISABLED,
        ARM_HEIGHT,
        ARM_ANGLE,
        FULL
    }

    public static CatalystArmMode mode = CatalystArmMode.DISABLED;

    // STATE HANDLING (optional)
    // 90 degrees
    public enum CatalystArmState {
        STOWED(Throttle.of(0), Inches.of(0), Degrees.of(90)),
        FLOOR(Throttle.of(1), Inches.of(5), Degrees.of(-45)),
        LOW_CATA(Throttle.of(1), Inches.of(8), Degrees.of(-20)),
        HIGH_CATA(Throttle.of(1), Inches.of(5), Degrees.of(45)),
        TRANSFER_READY(Throttle.of(0), Inches.of(10), Degrees.of(90)),
        TRANSFER(Throttle.of(-1), Inches.of(10), Degrees.of(90)),

        // CONSTANTS ONLY! DO NOT USE!
        MIN(Throttle.of(-1), Inches.of(0), Degrees.of(-90)),
        MAX(Throttle.of(1), Inches.of(10), Degrees.of(90));

        public final Voltage intakeThrottle;
        /**
         * 0 degrees = parallel with floor, 90 = towards ceiling, -90 = towards floor
         */
        public final Angle armAngle;
        public final Distance slideHeight;

        CatalystArmState(Voltage intakeThrottle, Distance slideHeight, Angle armAngle) {
            this.intakeThrottle = intakeThrottle;
            this.armAngle = armAngle;
            this.slideHeight = slideHeight;
        }
    }

    private CatalystArmState state = CatalystArmState.STOWED;

    // CONFIG VARS
    public static double INTAKE_PASSIVE_THROTTLE = 0.2;

    // MUTEXES
    public static class CatalystIntake extends SubsystemBase {}

    public CatalystIntake intakeMutex = new CatalystIntake();

    // INTERNAL VARS
    private Voltage actualThrottle = state.intakeThrottle;
    private CatalystArmState lastGrabState = CatalystArmState.FLOOR;

    // HARDWARE DEVICES
    private final Servo diffyLeft, diffyRight;
    private final CRServo intake;
    private final DigitalChannel intakeSensor;

    public CatalystArm(HardwareMap hardwareMap, Trigger catalystScoringTrigger) {
        this.diffyLeft = hardwareMap.servo.get("catalyst left diffy");
        this.diffyRight = hardwareMap.servo.get("catalyst right diffy");
        this.intake = hardwareMap.crservo.get("catalyst intake");

        this.intakeSensor = hardwareMap.digitalChannel.get("catalyst sensor");

        // mutex this so we can have manual outtake without dealing with state conflict
        intakeMutex.setDefaultCommand(new RunCommand(() ->
                this.intake.setPower(actualThrottle.into(Throttle)), intakeMutex));

        new Trigger(this.intakeSensor::getState)
                // when we grab, if not scoring, save last grab state and
                // set state to transfer ready (also serves as stowed)
                // if scoring, set state to high cata
                // also, set intake throttle to passive control (must be continuous)
                .whileActiveContinuous(() -> actualThrottle = Throttle.of(INTAKE_PASSIVE_THROTTLE))
                .whenActive(() ->
                        {
                            if (catalystScoringTrigger.get()) {
                                setState(CatalystArmState.HIGH_CATA);
                            } else {
                                lastGrabState = state;
                                setState(CatalystArmState.TRANSFER_READY);
                            }
                        }
                )
                // when we ungrab, if we just transferred, switch to the next cata grab state automatically
                // go off last grab to determine which cata state to switch to
                .whenInactive(() -> {
                    if (state == CatalystArmState.TRANSFER) {
                        switch (lastGrabState) {
                            case HIGH_CATA:
                                setState(CatalystArmState.LOW_CATA);
                                break;
                            case LOW_CATA:
                            case FLOOR: // default to HIGH for maximum flexibility
                                setState(CatalystArmState.HIGH_CATA);
                                break;
                        }
                    }
                })
                // check on rising edge
                .negate()
                .whileActiveContinuous(() -> actualThrottle = state.intakeThrottle);
    }

    public boolean hasCatalyst() {
        return this.intakeSensor.getState();
    }

    public Command outtake() {
        return new RunCommand(() -> this.intake.setPower(-0.5), intakeMutex);
    }

    public void setState(CatalystArmState state) {
        this.state = state;
        EnderLog.write("CatalystArm/events", "State updated to " + state.name());
        EnderLog.write("CatalystArm/targetHeight", state.slideHeight);
        EnderLog.write("CatalystArm/targetAngle", state.armAngle);
    }

    @Override
    public void periodic() {
        switch (mode) {
            case DISABLED:
                break; // don't run any hardware! the subsystem is off.
            case ARM_HEIGHT:
                executeIVK(state.slideHeight, Degrees.of(0.0));
                break;
            case ARM_ANGLE:
                executeIVK(Inches.of(0.0), state.armAngle);
                break;
            case FULL:
                executeIVK(state.slideHeight, state.armAngle);
                break;
        }

        // log any getters in periodic, as well as relevant internal vars
        EnderLog.write("CatalystArm/targetState", state.name());
        EnderLog.write("CatalystArm/targetHeight", state.slideHeight);
        EnderLog.write("CatalystArm/targetAngle", state.armAngle);
        EnderLog.write("CatalystArm/hasCatalyst", hasCatalyst());
        EnderLog.write("CatalystArm/attemptedThrottle", actualThrottle);
        EnderLog.write("CatalystArm/throttleIsOverriden", intakeMutex.getDefaultCommand().equals(intakeMutex.getCurrentCommand()));
    }

    public void executeIVK(Distance armHeight, Angle armAngle) {

        double distanceFactor = armHeight.div((Measure<? extends DistanceUnit>)
                CatalystArmState.MAX.slideHeight.minus(CatalystArmState.MIN.slideHeight));

        double angleFactor = armAngle.div((Measure<? extends AngleUnit>)
                CatalystArmState.MAX.armAngle.minus(CatalystArmState.MIN.armAngle));

        // flag normalization if it happens
        if (distanceFactor + angleFactor > 1 || distanceFactor - angleFactor < 0) {
            EnderLog.write("CatalystArm/warnings", "CatalystArm exceeded IVK limits with height "
                    + armHeight.into(Inches) + " inches and angle " + armAngle.into(Degrees) + " degrees.");
        }
        this.diffyLeft.setPosition(distanceFactor + angleFactor);
        this.diffyRight.setPosition(distanceFactor - angleFactor);
    }

}

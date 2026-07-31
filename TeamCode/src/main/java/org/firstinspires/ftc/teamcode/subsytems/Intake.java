package org.firstinspires.ftc.teamcode.subsytems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.*;
import com.seattlesolvers.solverslib.command.button.Trigger;
import dev.nextftc.units.measuretypes.Angle;
import dev.nextftc.units.measuretypes.Distance;
import dev.nextftc.units.measuretypes.Voltage;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.EnderLog;

import java.util.function.DoubleSupplier;

import static dev.nextftc.units.Units.*;
import static gay.zharel.fastlane.UnitsKt.*;

/**
 * This subsystem does something
 * descriptive stuff here
 */
@Config
public class Intake extends SubsystemBase {
    // MODE HANDLING (optional)
    // modes determine *how* a subsystem runs. states are descriptions of a subsystem's target state (control theory state).
    // modes are meant for bringup/tuning. states are meant for systems operating at full capacity (mode.FULL).

    // the intake has no modes because its too simple lol

    // STATE HANDLING
    public enum IntakeState {
        STOWED(Degrees.of(INTAKE_STOWED_DEGREES), Throttle.of(0.0)),
        TRANSFER(Degrees.of(INTAKE_STOWED_DEGREES), Throttle.of(1.0)),
        DEPLOYED(Degrees.of(0.0), Throttle.of(1.0)),
        REVERSE(Degrees.of(0.0), Throttle.of(-1.0));

        /**
         * 0.0 is deployed angle
         */
        public final Angle pivotAngle;
        public final Voltage intakeThrottle;

        IntakeState(Angle angle, Voltage throttle) {
            this.pivotAngle = angle;
            this.intakeThrottle = throttle;
        }
    }
    private IntakeState state = IntakeState.STOWED;

    // CONFIG VARS
    public static double INTAKE_STOWED_DEGREES = 0.0;
    public static double TRANSFER_OSCILLATION_SECONDS = 2.0;
    public static double INTAKE_JAM_AMPS = 6.0;
    public static double JAM_OSCILLATION_SECONDS = 0.5;
    public static double SERVO_MIN_RADIANS = -0.5;
    public static double SERVO_MAX_RADIANS = 2.0;

    // HARDWARE DEVICES
    private final DcMotorEx intake;
    private final Servo intakePivot;

    public Intake(HardwareMap hardwareMap, Trigger catalystTrigger) {
        this.intake = hardwareMap.get(DcMotorEx.class, "intake motor");
        this.intakePivot = hardwareMap.servo.get("intake pivot");

        this.intake.setCurrentAlert(INTAKE_JAM_AMPS, CurrentUnit.AMPS);

        // transfer logic
        catalystTrigger
                .whileActiveOnce(
                        new RepeatCommand(
                                setStateCommand(IntakeState.TRANSFER)
                                        .andThen(
                                                new WaitCommand((long)TRANSFER_OSCILLATION_SECONDS * 1000),
                                                setStateCommand(IntakeState.DEPLOYED),
                                                new WaitCommand((long) TRANSFER_OSCILLATION_SECONDS * 1000)
                                        )
                        )
                )
                .whenInactive(setStateCommand(IntakeState.DEPLOYED));

        // antijam logic
        new Trigger(this.intake::isOverCurrent)
                .and(new Trigger(() -> this.state == IntakeState.DEPLOYED))
                .whileActiveOnce(
                        new RepeatCommand(
                                setStateCommand(IntakeState.REVERSE)
                                        .andThen(
                                                new WaitCommand((long)JAM_OSCILLATION_SECONDS * 1000),
                                                setStateCommand(IntakeState.DEPLOYED),
                                                new WaitCommand((long) JAM_OSCILLATION_SECONDS * 1000)
                                        )
                        )
                )
                .whenInactive(setStateCommand(IntakeState.DEPLOYED));

        setDefaultCommand(new RunCommand(
                () -> {
                    this.intake.setPower(state.intakeThrottle.into(Throttle));
                    this.intakePivot.setPosition(getServoPosition(state.pivotAngle));
                },
                this
        ));
    }

    private double getServoPosition(Angle pivotAngle) {
        return pivotAngle.into(Radians) - SERVO_MIN_RADIANS / (SERVO_MAX_RADIANS - SERVO_MIN_RADIANS);
    }

    public void setState(IntakeState state) {
        this.state = state;
    }

    public Command setStateCommand(IntakeState state) {
        return new InstantCommand(() -> this.state = state);
    }

    @Override
    public void periodic() {
        // log any getters in periodic, as well as relevant internal vars
        EnderLog.write("intakePivotAngle", state.pivotAngle);
        EnderLog.write("intakeThrottle", state.intakeThrottle);
    }

}

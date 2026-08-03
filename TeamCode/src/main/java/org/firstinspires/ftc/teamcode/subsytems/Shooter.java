package org.firstinspires.ftc.teamcode.subsytems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.button.Trigger;
import dev.nextftc.units.measuretypes.Angle;
import dev.nextftc.units.measuretypes.AngularVelocity;
import dev.nextftc.units.measuretypes.Distance;
import org.firstinspires.ftc.teamcode.util.BangBangController;
import org.firstinspires.ftc.teamcode.util.EnderLog;
import org.firstinspires.ftc.teamcode.util.ServoUtil;

import java.util.function.DoubleSupplier;

import static dev.nextftc.units.Units.*;
import static gay.zharel.fastlane.UnitsKt.*;

/**
 * shoots artifacts or something
 */
@Config
public class Shooter extends SubsystemBase {
    // MODE HANDLING (optional)
    // modes determine *how* a subsystem runs. states are descriptions of a subsystem's target state (control theory state).
    // modes are meant for bringup/tuning. states are meant for systems operating at full capacity (mode.FULL).
    public enum ShooterMode {
        DISABLED,
        TURRET,
        FLYWHEEL,
        HOOD,
        TUNING,
        FULL
    }
    public static ShooterMode mode = ShooterMode.DISABLED;

    // STATE HANDLING (optional)
    public static class ShooterState {
        /**
         * 0 is lowest possible position
         */
        public final Angle hoodAngle;
        /**
         * -180 to 180, unwrapped
         */
        public final Angle turretAngle;
        public final AngularVelocity flywheelAngVel;

        ShooterState(Angle hoodAngle, Angle turretAngle, AngularVelocity flywheelAngVel) {
            this.hoodAngle = hoodAngle;
            this.turretAngle = turretAngle;
            this.flywheelAngVel = flywheelAngVel;
        }
    }
    private ShooterState state = new ShooterState(Radians.of(0.0), Radians.of(0.0), RotationsPerMinute.of(0.0));

    // CONFIG VARS
    public static double TURRET_MIN_DEGREES = -175;
    public static double TURRET_MAX_DEGREES = 180;
    public static double HOOD_MIN_DEGREES = 0.0;
    public static double HOOD_MAX_DEGREES = 30.0;

    public static double FLYWHEEL_THROTTLE_PER_RPM = 0.001;
    public static double FLYWHEEL_REVOLUTIONS_PER_TICK = 1.0 / 17.0;
    public static double FLYWHEEL_BANGBANG_THROTTLE = 0.2;
    // MUTEXES (optional)
    public static class Turret extends SubsystemBase {}
    public static class Flywheel extends SubsystemBase {}
    public static class Hood extends SubsystemBase {}
    public Turret turretMutex = new Turret();
    public Flywheel flywheelMutex = new Flywheel();
    public Hood hoodMutex = new Hood();

    // INTERNAL VARS
    private final BangBangController<AngularVelocity> flywheelController = new BangBangController<>(FLYWHEEL_BANGBANG_THROTTLE, 0);
    private final ServoUtil turretUtil, hoodUtil;

    // HARDWARE DEVICES
    private final DcMotorEx flywheel;
    private final Servo turret, hood;

    public Shooter(HardwareMap hardwareMap) {
        this.flywheel = hardwareMap.get(DcMotorEx.class, "flywheel motor");

        this.turret = hardwareMap.servo.get("turret servo");
        this.hood = hardwareMap.servo.get("hood servo");

        this.turretUtil = new ServoUtil(Degrees.of(TURRET_MIN_DEGREES), Degrees.of(TURRET_MAX_DEGREES));
        this.hoodUtil = new ServoUtil(Degrees.of(HOOD_MIN_DEGREES), Degrees.of(HOOD_MAX_DEGREES));

        // it's probably better to put these in periodic but this is nice too
        switch (mode) {
            case DISABLED:
                break;
            case TURRET:
                turretMutex.setDefaultCommand(
                        new RunCommand(() -> {
                            this.turret.setPosition(turretUtil.getPosition(state.turretAngle));
                        }, turretMutex)
                );
                break;
            case HOOD:
                hoodMutex.setDefaultCommand(
                        new RunCommand(() -> {
                            this.hood.setPosition(hoodUtil.getPosition(state.hoodAngle));
                        }, hoodMutex)
                );
                break;
            case FLYWHEEL:
                flywheelMutex.setDefaultCommand(
                        new RunCommand(() -> {
                            this.flywheel.setPower(state.flywheelAngVel.into(RotationsPerMinute) * FLYWHEEL_THROTTLE_PER_RPM
                                    + flywheelController.get(getFlywheelAngVel(), state.flywheelAngVel));
                        })
                );
                break;
            case TUNING:
            case FULL:
                turretMutex.setDefaultCommand(
                        new RunCommand(() -> {
                            this.turret.setPosition(turretUtil.getPosition(state.turretAngle));
                        }, turretMutex)
                );
                hoodMutex.setDefaultCommand(
                        new RunCommand(() -> {
                            this.hood.setPosition(hoodUtil.getPosition(state.hoodAngle));
                        }, hoodMutex)
                );
                flywheelMutex.setDefaultCommand(
                        new RunCommand(() -> {
                            this.flywheel.setPower(state.flywheelAngVel.into(RotationsPerMinute) * FLYWHEEL_THROTTLE_PER_RPM
                                    + flywheelController.get(getFlywheelAngVel(), state.flywheelAngVel));
                        })
                );
                break;

        }
    }

    public void setState(ShooterState state) {
        this.state = state;
    }

    public Command setStateCommand(ShooterState state) {
        return new InstantCommand(() -> this.state = state);
    }

    public AngularVelocity getFlywheelAngVel() {
        return RotationsPerSecond.of(this.flywheel.getVelocity() * FLYWHEEL_REVOLUTIONS_PER_TICK);
    }

    public ShooterState getState() {
        return new ShooterState(state.hoodAngle, state.turretAngle, getFlywheelAngVel());
    }

    @Override
    public void periodic() {
        if (mode == ShooterMode.FULL) {
            // calcs go here
        }

        // log any getters in periodic, as well as relevant internal vars
        EnderLog.write("targetState", state);
        EnderLog.write("currentState", getState());
    }
}

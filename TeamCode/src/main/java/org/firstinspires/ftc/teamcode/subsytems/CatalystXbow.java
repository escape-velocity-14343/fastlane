package org.firstinspires.ftc.teamcode.subsytems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import dev.nextftc.control.geometry.Pose2d;
import dev.nextftc.units.measuretypes.Angle;
import dev.nextftc.units.measuretypes.Distance;
import org.firstinspires.ftc.teamcode.util.EnderLog;
import org.firstinspires.ftc.teamcode.util.ServoUtil;

import java.util.function.DoubleSupplier;

import static dev.nextftc.units.Units.*;

/**
 * shoots the catalysts or something
 */
@Config
public class CatalystXbow extends SubsystemBase {
    // MODE HANDLING (optional)
    // modes determine *how* a subsystem runs. states are descriptions of a subsystem's target state (control theory state).
    // modes are meant for bringup/tuning. states are meant for systems operating at full capacity (mode.FULL).
    public enum XbowMode {
        DISABLED,
        TUNING, // disables autoaim (incl rotation override)
        FULL
    }
    public static XbowMode mode = XbowMode.DISABLED;

    // STATE HANDLING (optional)
    public static class XbowState {

        /**
         * 0 is parallel with tiles, +90 is straight up
         */
        public final Angle pitch;
        public final Pose2d target;

        XbowState(Angle pitch, Pose2d target) {
            this.pitch = pitch;
            this.target = target;
        }
    }
    public static XbowState STOWED = new XbowState(Radians.of(0.0), Pose2d.zero);
    public static XbowState TRANSFER = new XbowState(Radians.of(90.0), Pose2d.zero);
    public static XbowState MAX = new XbowState(Radians.of(90.0), Pose2d.zero);
    public static XbowState MIN = new XbowState(Radians.of(-30.0), Pose2d.zero);
    private XbowState state = STOWED;
    // CONFIG VARS
    public static double LAUNCH_RETRACTED_POS = 0.0;
    public static double LAUNCH_LAUNCH_POS = 0.5;
    public static double LAUNCH_MILLIS = 200;

    // HARDWARE DEVICES
    private final Servo launch, pitch;
    private final ServoUtil pitchUtil;

    public CatalystXbow(HardwareMap hardwareMap) {
        this.launch = hardwareMap.servo.get("choo choo");
        this.pitch = hardwareMap.servo.get("xbow pitch");

        this.pitchUtil = new ServoUtil(MIN.pitch, MAX.pitch);
    }

    public void setState(XbowState state) {
        this.state = state;
    }

    @Override
    public void periodic() {
        switch (mode) {
            case DISABLED:
                break; // don't run any hardware! the subsystem is off.
            case TUNING: // do stuff
            case FULL:
                pitch.setPosition(pitchUtil.getPosition(state.pitch));
                break;
        }

        // log any getters in periodic, as well as relevant internal vars
        EnderLog.write("CatalystXbow/pitchDegrees", state.pitch.into(Degrees));
        EnderLog.write("CatalystXbow/targetPose", state.target);
    };

    public Command launch() {
        return new InstantCommand(() -> this.launch.setPosition(LAUNCH_LAUNCH_POS))
                .andThen(new WaitCommand((long) LAUNCH_MILLIS), new InstantCommand(() -> this.launch.setPosition(LAUNCH_RETRACTED_POS))
                        .alongWith(
                                new InstantCommand(() -> setState(TRANSFER)) // prep for next
                        ));
    }

}

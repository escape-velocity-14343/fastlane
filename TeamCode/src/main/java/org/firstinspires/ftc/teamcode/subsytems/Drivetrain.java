package org.firstinspires.ftc.teamcode.subsytems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.button.Trigger;
import gay.zharel.fastlane.PoseVoltage2d;
import kotlin.NotImplementedError;
import org.firstinspires.ftc.teamcode.util.EnderLog;

import java.util.function.DoubleSupplier;

@Config
public class Drivetrain extends SubsystemBase {

    // state
    public enum DrivetrainState {
        DISABLED,
        NO_SUSPENSION,
        FULL
    }
    public static DrivetrainState state = DrivetrainState.DISABLED;

    public static double STABILIZED_POSITION = 0.0;
    public static double RELEASED_POSITION = 0.0;

    // mutexes for requirements
    public class Suspension extends SubsystemBase {}
    public Suspension suspension = new Suspension();

    private final DcMotorEx lf, lb, rf, rb;
    private final Servo stabilizer;
    private final DoubleSupplier yawSupplier;

    public Drivetrain(HardwareMap hardwareMap, Trigger terrainTrigger, DoubleSupplier yawSupplier) {
        super();
        this.lf = (DcMotorEx) hardwareMap.dcMotor.get("left front drive");
        this.lb = (DcMotorEx) hardwareMap.dcMotor.get("left back drive");
        this.rf = (DcMotorEx) hardwareMap.dcMotor.get("right front drive");
        this.rb = (DcMotorEx) hardwareMap.dcMotor.get("right back drive");

        this.stabilizer = hardwareMap.servo.get("drive stabilizer");

        this.yawSupplier = yawSupplier;

        terrainTrigger
                .whenActive(() -> {setStabilizer(true);})
                .whenInactive(() -> {setStabilizer(false);});
    }

    // fc drive
    public void drive(PoseVoltage2d voltage) {
        throw new NotImplementedError("bro i cant be bothered");
    }

    public void setStabilizer(boolean stabilize) {
        if (state == DrivetrainState.DISABLED) {
            return;
        }
        this.stabilizer.setPosition(stabilize ? STABILIZED_POSITION : RELEASED_POSITION);
        EnderLog.write("Stabilizer", stabilize);
    }
}

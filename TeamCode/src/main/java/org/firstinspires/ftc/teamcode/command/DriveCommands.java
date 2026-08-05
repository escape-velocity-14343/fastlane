package org.firstinspires.ftc.teamcode.command;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.RunCommand;
import dev.nextftc.control.geometry.Pose2d;
import dev.nextftc.control.geometry.Rotation2d;
import dev.nextftc.control.geometry.Twist2d;
import dev.nextftc.control.geometry.Vector2d;
import dev.nextftc.units.measuretypes.Angle;
import dev.nextftc.units.measuretypes.Voltage;
import gay.zharel.fastlane.Controller;
import gay.zharel.fastlane.PoseVoltage2d;
import org.firstinspires.ftc.teamcode.subsytems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsytems.Vision;

import java.util.function.DoubleSupplier;

import static dev.nextftc.units.Units.Radians;
import static gay.zharel.fastlane.UnitsKt.Throttle;


public class DriveCommands {

    public static Controller<Pose2d, PoseVoltage2d> drivetrainController;

    public static Controller<Angle, Voltage> headingController = new Controller<Angle, Voltage>() {
        @Override
        public Voltage get(Angle pv, Angle sp) {
            return (Voltage) drivetrainController.get(Pose2d.zero.plus(new Twist2d(0, 0, pv.into(Radians))),
                    Pose2d.zero.plus(new Twist2d(0, 0, sp.into(Radians)))).angVolt;
        }
    };

    public static Command getDriveDefaultCommand(DoubleSupplier forward, DoubleSupplier right, DoubleSupplier ccw, Drivetrain dt) {
        return new RunCommand(() -> {
            dt.drive(
                    new PoseVoltage2d(
                            new Vector2d<>(
                                    Throttle.of(forward.getAsDouble()),
                                    Throttle.of(right.getAsDouble())
                            ),
                            Throttle.of(ccw.getAsDouble())
                    )
            );
        }, dt);
    }

    public static Command getTurnToPointCommand(DoubleSupplier forward, DoubleSupplier right, Pose2d point,
                                                Controller<Angle, Voltage> angleController, Vision vision, Drivetrain dt) {
        return getDriveDefaultCommand(forward, right, () -> {
            Angle angleTo = Radians.of(point.position.minus(vision.getVisionPose().position).angle().toDouble());
            return angleController.get(Radians.of(vision.getYaw()), angleTo).into(Throttle);
        }, dt);
    }

}

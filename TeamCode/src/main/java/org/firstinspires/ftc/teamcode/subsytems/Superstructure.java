package org.firstinspires.ftc.teamcode.subsytems;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import dev.nextftc.control.geometry.Pose2d;
import dev.nextftc.control.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.command.DriveCommands;

import java.util.function.DoubleSupplier;

import static dev.nextftc.units.Units.Degrees;

@Config
public class Superstructure {
    public static double CATALYST_TRANSFER_MILLIS = 500;
    public static Pose2d XBOW_TARGET = new Pose2d(0.0, -50.0, Rotation2d.zero); // todo: alliance flipping
    public Command transferToXbow(DoubleSupplier forward, DoubleSupplier right, Vision vision, Drivetrain dt, CatalystXbow xbow, CatalystArm arm) {
        return new ParallelCommandGroup(
                new InstantCommand(() -> arm.setState(CatalystArm.CatalystArmState.TRANSFER)).andThen(
                        new WaitCommand((long) CATALYST_TRANSFER_MILLIS)
                ), // transfer

                // xbow state
                new InstantCommand(() -> xbow.setState(new CatalystXbow.XbowState(Degrees.of(0.0), XBOW_TARGET))),

                // drivetrain override
                DriveCommands.getTurnToPointCommand(
                        forward,
                        right,
                        XBOW_TARGET,
                        DriveCommands.headingController,
                        vision,
                        dt
                )
        );
    }
}

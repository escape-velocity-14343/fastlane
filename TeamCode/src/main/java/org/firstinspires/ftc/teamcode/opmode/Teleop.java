package org.firstinspires.ftc.teamcode.opmode;

import android.os.Trace;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import dev.nextftc.control.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.command.DriveCommands;
import org.firstinspires.ftc.teamcode.subsytems.*;

import static dev.nextftc.units.Units.Inches;

@TeleOp
public class Teleop extends LinearOpMode {

    private GamepadEx driver, operator;
    private Drivetrain drivetrain;
    private CatalystArm catalystArm;
    private CatalystXbow catalystXbow;
    private DyeRotor dyeRotor;
    private Intake intake;
    private Shooter shooter;
    private Superstructure superstructure;
    private Vision vision;

    private Trigger terrainTrigger, catalystScoringTrigger, catalystDescoringTrigger;


    @Override
    public void runOpMode() throws InterruptedException {

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        vision = new Vision(hardwareMap);

        terrainTrigger = new Trigger(() -> vision.getVisionPose().position.x.abs(Inches) < 10
                || vision.getVisionPose().position.y.abs(Inches) < 10);
        catalystScoringTrigger = new Trigger(() -> vision.getVisionPose().position.minus(new Vector2d<>(
                Inches.of(0.0), Inches.of(-72.0)
        )).sqrNorm() < 30 * 30);
        catalystDescoringTrigger = new Trigger(() -> vision.getVisionPose().position.minus(new Vector2d<>(
                Inches.of(0.0), Inches.of(72.0)
        )).sqrNorm() < 30 * 30);

        drivetrain = new Drivetrain(hardwareMap, terrainTrigger, vision::getYaw);
        catalystArm = new CatalystArm(hardwareMap, catalystScoringTrigger);
        catalystXbow = new CatalystXbow(hardwareMap);
        dyeRotor = new DyeRotor(hardwareMap);
        intake = new Intake(hardwareMap, catalystScoringTrigger.or(catalystDescoringTrigger));
        shooter = new Shooter(hardwareMap);
        superstructure = new Superstructure();

        // drive: standard
        drivetrain.setDefaultCommand(DriveCommands.getDriveDefaultCommand(
                driver::getLeftY,
                driver::getLeftX,
                driver::getRightX,
                drivetrain
        ));

        // catalyst

        // groundtake
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05)
                .whenActive(() -> catalystArm.setState(CatalystArm.CatalystArmState.FLOOR));
        // outtake
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05)
                .whileActiveContinuous(catalystArm.outtake());

        // high low transfer launch
        driver.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(() -> catalystArm.setState(CatalystArm.CatalystArmState.HIGH_CATA));
        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> catalystArm.setState(CatalystArm.CatalystArmState.LOW_CATA));

        driver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> superstructure.transferToXbow(
                        driver::getLeftY,
                        driver::getLeftX,
                        vision,
                        drivetrain,
                        catalystXbow,
                        catalystArm
                ));

        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> catalystXbow.launch());

        waitForStart();

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
        }
    }
}

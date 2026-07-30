package org.firstinspires.ftc.teamcode.subsytems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.button.Trigger;
import dev.nextftc.units.measuretypes.Distance;
import org.firstinspires.ftc.teamcode.util.EnderLog;

import java.util.function.DoubleSupplier;

import static dev.nextftc.units.Units.*;

@Config
public class TemplateSubsystem extends SubsystemBase {
    // STATE HANDLING
    public enum ExampleState {
        DISABLED,
        STATE1,
        STATE2,
        STATE3
    }
    public ExampleState state = ExampleState.DISABLED;

    // CONFIG VARS
    public static double EXAMPLE_CONFIG_VAR = 0.0;

    // MUTEXES

    public class InnerSystem1 extends SubsystemBase {}
    public class InnerSystem2 extends SubsystemBase {}
    public InnerSystem1 system1 = new InnerSystem1();
    public InnerSystem2 system2 = new InnerSystem2();

    // INTERNAL VARS
    private double internalSomething = 0.0;
    private Distance internalSomething2 = Inches.of(10.0);

    // HARDWARE DEVICES
    private DcMotorEx internalMotor, internalMotor2;

    public TemplateSubsystem(HardwareMap hardwareMap, Trigger exampleTrigger, DoubleSupplier exampleSupplier) {
        this.internalMotor = hardwareMap.get(DcMotorEx.class, "super cool motor");
        this.internalMotor2 = hardwareMap.get(DcMotorEx.class, "less cool motor");

        exampleTrigger
                .whenActive(() -> EnderLog.write("bruh", "i did a thing " + exampleSupplier.getAsDouble()));
    }

    @Override
    public void periodic() {
        switch (state) {
            case DISABLED:
                break; // don't run any hardware! the subsystem is off.
            case STATE1: // do stuff
                openLoopMotor();
                break;
            case STATE2:
                openLoopMotor();
                closedLoopMotor();
                break;
            case STATE3:
                closedLoopMotor();
                break;
        }

        // log any getters in periodic, as well as relevant internal vars
        EnderLog.write("descriptiveName", internalSomething2);
        EnderLog.write("distance", getDistance());
    }

    // state open/closed loop stuff
    // naming: non-compensating (not grav or accel) ff is open loop, grav and accel are gravLoop and accelLoop,
    // anything with feedback is closedLoop
    private void openLoopMotor() {
        this.internalMotor.setPower(0.5);
    }

    private void closedLoopMotor() {
        this.internalMotor.setPower(0.7);
    }

    public Distance getDistance() {
        return Inches.of(this.internalMotor.getCurrentPosition() * EXAMPLE_CONFIG_VAR);
    }

    public void setSomething(boolean something) {
        this.internalSomething = something ? 5.0 : 0.0;
        // log any variable assignments in setters directly in the setter
        EnderLog.write("something", something);
        EnderLog.write("somethingValue", internalSomething);
    }

}

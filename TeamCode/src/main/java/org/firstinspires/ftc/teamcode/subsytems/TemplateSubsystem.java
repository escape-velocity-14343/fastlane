package org.firstinspires.ftc.teamcode.subsytems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.button.Trigger;
import dev.nextftc.units.measuretypes.Angle;
import dev.nextftc.units.measuretypes.Distance;
import org.firstinspires.ftc.teamcode.util.EnderLog;

import java.util.function.DoubleSupplier;

import static dev.nextftc.units.Units.*;

/**
 * This subsystem does something
 * descriptive stuff here
 */
@Config
public class TemplateSubsystem extends SubsystemBase {
    // MODE HANDLING (optional)
    // modes determine *how* a subsystem runs. states are descriptions of a subsystem's target state (control theory state).
    // modes are meant for bringup/tuning. states are meant for systems operating at full capacity (mode.FULL).
    public enum ExampleMode {
        DISABLED,
        MODE1,
        MODE2,
        FULL
    }
    public static CatalystXbowMode mode = CatalystXbowMode.DISABLED;

    // STATE HANDLING (optional)
    public enum ExampleState {
        STATE1(Radians.of(0.0), Inches.of(10.0)),
        STATE2(Radians.of(5.0), Inches.of(10.0));

        public final Angle somethingAngle;
        public final Distance somethingDistance;

        ExampleState(Angle angle, Distance distance) {
            this.somethingDistance = distance;
            this.somethingAngle = angle;
        }
    }
    private ExampleState state = ExampleState.STATE1;

    // CONFIG VARS
    public static double EXAMPLE_CONFIG_VAR = 0.0;

    // MUTEXES (optional)
    public static class InnerSystem1 extends SubsystemBase {}
    public static class InnerSystem2 extends SubsystemBase {}
    public InnerSystem1 system1 = new InnerSystem1();
    public InnerSystem2 system2 = new InnerSystem2();

    // INTERNAL VARS
    private double internalSomething = 0.0;
    private Distance internalSomething2 = Inches.of(10.0);

    // HARDWARE DEVICES
    private final DcMotorEx internalMotor, internalMotor2;

    public TemplateSubsystem(HardwareMap hardwareMap, Trigger exampleTrigger, DoubleSupplier exampleSupplier) {
        this.internalMotor = hardwareMap.get(DcMotorEx.class, "super cool motor");
        this.internalMotor2 = hardwareMap.get(DcMotorEx.class, "less cool motor");

        exampleTrigger
                .whenActive(() -> EnderLog.write("bruh", "i did a thing " + exampleSupplier.getAsDouble()));
    }

    public void setState(ExampleState state) {
        this.state = state;
    }

    @Override
    public void periodic() {
        switch (mode) {
            case DISABLED:
                break; // don't run any hardware! the subsystem is off.
            case MODE1: // do stuff
                openLoopMotor();
                break;
            case MODE2:
                closedLoopMotor();
                break;
            case FULL:
                openLoopMotor();
                closedLoopMotor();
                break;
        }

        // log any getters in periodic, as well as relevant internal vars
        EnderLog.write("descriptiveName", internalSomething2);
        EnderLog.write("distance", getDistance());
    }

    // mode open/closed loop stuff
    // naming: non-compensating (not grav or accel) ff is open loop, grav and accel are gravLoop and accelLoop,
    // anything with feedback is closedLoop
    private void openLoopMotor() {
        this.internalMotor.setPower(0.5);
    }

    private void closedLoopMotor() {
        // do something with the state
        setSomething(state.somethingDistance.compareTo(Inches.of(5)) > 0);
        this.internalMotor.setPower(0.7); // yes this is not feedback no i don't care make it feedback irl
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

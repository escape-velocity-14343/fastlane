package org.firstinspires.ftc.teamcode.subsytems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import dev.nextftc.units.measuretypes.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.BangBangController;
import org.firstinspires.ftc.teamcode.util.EnderLog;

import static dev.nextftc.units.Units.*;

@Config
public class DyeRotor extends SubsystemBase {
    // state
    public enum DyeRotorState {
        DISABLED,
        OPEN_LOOP_ONLY,
        TRANSFER_FEEDBACK,
        FULL
    }
    public static DyeRotorState state = DyeRotorState.DISABLED;
    public static double DYE_ROTOR_STALL_AMPS = 6.0;
    public static double DYE_ROTOR_THROTTLE = 1.0;
    public static double DYE_ROTOR_STALL_THROTTLE = 1.0;
    public static double TRANSFER_BANG_BANG_THROTTLE = 0.4;
    public static double TRANSFER_TARGET_RPM = 600;
    public static double TRANSFER_KV = 0.001;
    public static double TRANSFER_ROTATIONS_PER_TICK = 0.1;

    private final DcMotorEx dye, transfer;
    private final BangBangController<AngularVelocity> transferBangBang = new BangBangController<>(TRANSFER_BANG_BANG_THROTTLE, 0.0);

    public DyeRotor(HardwareMap hardwareMap) {
        super();
        this.dye = (DcMotorEx) hardwareMap.dcMotor.get("dye rotor");
        this.transfer = (DcMotorEx) hardwareMap.dcMotor.get("transfer");

        this.dye.setCurrentAlert(DYE_ROTOR_STALL_AMPS, CurrentUnit.AMPS);
    }

    @Override
    public void periodic() {
        switch (state) {
            case DISABLED:
                break;
            case OPEN_LOOP_ONLY:
                openLoopDye();
                openLoopTransfer();
                break;
            case FULL:
                openLoopDye();
            case TRANSFER_FEEDBACK:
                closedLoopTransfer();
                break;
        }

        EnderLog.write("TransferAngularVelocity", getTransferAngVel());
    }

    public AngularVelocity getTransferAngVel() {
        // cached read
        return RotationsPerSecond.of(this.transfer.getVelocity() * TRANSFER_ROTATIONS_PER_TICK);
    }

    private void openLoopDye() {
        this.dye.setPower(dye.isOverCurrent() ? DYE_ROTOR_STALL_THROTTLE : DYE_ROTOR_THROTTLE);
    }

    private void openLoopTransfer() {
        this.transfer.setPower(TRANSFER_TARGET_RPM * TRANSFER_KV); // just ff it
    }

    private void closedLoopTransfer() {
        this.transfer.setPower(TRANSFER_TARGET_RPM * TRANSFER_KV
                + transferBangBang.get(getTransferAngVel(), RotationsPerMinute.of(TRANSFER_TARGET_RPM)));
    }

}

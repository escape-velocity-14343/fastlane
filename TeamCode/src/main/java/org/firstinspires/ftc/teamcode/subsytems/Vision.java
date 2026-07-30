package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import dev.nextftc.control.geometry.Pose2d;
import kotlin.NotImplementedError;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.EnderLog;

/**
 * vision or something
 * there would be a vslam impl here trust
 * also owns the pinpoint because it wants orientation
 */
public class Vision extends SubsystemBase {

    private GoBildaPinpointDriver pinpoint;

    public Vision(HardwareMap hardwareMap) {
        super(); // self-register
        this.pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }

    @Override
    public void periodic() {
        this.pinpoint.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);
        EnderLog.write("PinpointHeadingRad", getYaw());
    }

    /**
     * @return in rad
     */
    public double getYaw() {
        return this.pinpoint.getHeading(AngleUnit.RADIANS);
    }

    public Pose2d getVisionPose() {
        throw new NotImplementedError("i aint coding vslam for allat");
    }

}

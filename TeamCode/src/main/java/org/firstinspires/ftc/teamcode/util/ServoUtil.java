package org.firstinspires.ftc.teamcode.util;

import dev.nextftc.units.Measure;
import dev.nextftc.units.measuretypes.Angle;
import dev.nextftc.units.unittypes.AngleUnit;

public class ServoUtil {
    private final Angle minAngle, maxAngle;

    public ServoUtil(Angle minAngle, Angle maxAngle) {
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
    }

    public static double getPosition(Angle minAngle, Angle maxAngle, Angle requestedAngle) {
        return new ServoUtil(minAngle, maxAngle).getPosition(requestedAngle);
    }

    public double getPosition(Angle angle) {
        return angle
                .minus(minAngle)
                .div((Measure<? extends AngleUnit>) maxAngle.minus(minAngle)); // stupid typecast
    }

}

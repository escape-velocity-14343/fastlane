package org.firstinspires.ftc.teamcode.util;

import gay.zharel.fastlane.Controller;

public class BangBangController<X extends Comparable<? super X>> implements Controller<X, Double> {
    private final double high, low;

    public BangBangController(double high, double low) {
        this.high = high;
        this.low = low;
    }

    @Override
    public Double get(X pv, X sp) {
        int compare = sp.compareTo(pv);
        if (compare > 0) {
            return high;
        } else if (compare == 0) {
            return 0.0;
        } else {
            return low;
        }
    }
}

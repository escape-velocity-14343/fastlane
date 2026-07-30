package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import gay.zharel.fateweaver.flight.FateLogManager;
import gay.zharel.fateweaver.log.FateLogWriter;

/**
 * Singleton logger class
 */
public class EnderLog {

    public static EnderLog INSTANCE = new EnderLog();

    public FateLogWriter logWriter;

    private EnderLog() {}

    public void start(OpMode opmode) {
        if (this.logWriter != null) {
            this.logWriter.close();
        }

        this.logWriter = FateLogManager.INSTANCE.start(opmode.getClass().getSimpleName() + "-" + System.nanoTime());
    }

    public static void write(String channel, Object obj) {
        INSTANCE.logWriter.write(channel, obj);
    }

}

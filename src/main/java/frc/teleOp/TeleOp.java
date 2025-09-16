package frc.teleOp;

import frc.robot.subsystems.outtake.Outtake;

public class TeleOp {
    private static TeleOp instance;
    private TeleOp() {
    }
    public static TeleOp getInstance() {
        if (instance == null) {
            instance = new TeleOp();
        }
        return instance;
    }

    public void init() {
        Outtake.getInstance().init();
    }
    public void loop() {
        Outtake.getInstance().update();
    }
}

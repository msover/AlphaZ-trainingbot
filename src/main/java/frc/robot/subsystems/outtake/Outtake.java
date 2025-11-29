package frc.robot.subsystems.outtake;

import frc.robot.subsystems.outtake.lift.Lift;

public class Outtake {
    private static Outtake instance;
    public static Outtake getInstance(){
        if(instance == null) {
            instance = new Outtake();
        }
        return instance;
    }

    public void init() {
        Lift.getInstace().init();
    }
    public void update() {
        Lift.getInstace().update();
    }
}

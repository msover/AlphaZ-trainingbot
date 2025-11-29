package frc.robot.subsystems.outtake;

import edu.wpi.first.wpilibj.PS5Controller;
import frc.robot.subsystems.outtake.lift.Lift;

public class Outtake {
    private static Outtake instance;
    public static Outtake getInstance(){
        if(instance == null) {
            instance = new Outtake();
        }
        return instance;
    }

    public void init(PS5Controller gamepad) {
        Lift.getInstace().initPID();
    }
    public void update() {
        Lift.getInstace().update();
    }
}

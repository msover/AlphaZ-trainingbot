package frc.robot.subsystems.outtake;

import frc.robot.utils.Constants;
import frc.robot.utils.Hardware;
import frc.robot.utils.PID.Outtake.LiftPID;
import frc.robot.utils.wrappers.Gamepad;

public class Outtake {
    private static Outtake instance;
    private Outtake() {
    }
    public static Outtake getInstance() {
        if (instance == null) {
            instance = new Outtake();
        }
        return instance;
    }
    
    private void liftAction() {
        //Placeholder values
        if(Gamepad.getInstance().get1().getAButton()) {
            Hardware.Outtake.Lift.targetPos = Constants.Outtake.Lift.liftUp;
        }
        if(Gamepad.getInstance().get1().getBButton()) {
            Hardware.Outtake.Lift.targetPos = Constants.Outtake.Lift.liftDown;
        }
    }
    public void init() {
        Hardware.Outtake.Lift.targetPos = Constants.Outtake.Lift.liftDown;
        LiftPID.init();
    }
    public void update() {
        LiftPID.update(Hardware.Outtake.Lift.liftMotor, Hardware.Outtake.Lift.liftEncoder, Hardware.Outtake.Lift.targetPos);
        liftAction();
    }
}

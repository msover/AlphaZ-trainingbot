package frc.robot.subsystems.outtake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.hardware.Hardware;
import frc.robot.utils.PID.OuttakePID;
import frc.robot.utils.PID.OuttakePID.LiftPID;
import frc.robot.utils.constants.Constants;
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
    private LiftPID LiftPID = new OuttakePID().new LiftPID(Hardware.Outtake.Lift.liftMotor, Hardware.Outtake.Lift.liftEncoder);
    private void setLiftState() {
        //Placeholder values
        if(Gamepad.get1().getAButton()) {
            Hardware.Outtake.Lift.targetPos = Constants.Outtake.Lift.liftUp;
        }
        if(Gamepad.get1().getBButton()) {
            Hardware.Outtake.Lift.targetPos = Constants.Outtake.Lift.liftDown;
        }
    }
    public void init() {
        Hardware.Outtake.Lift.targetPos = Constants.Outtake.Lift.liftDown;
    }
    public void update() {
        LiftPID.update(Hardware.Outtake.Lift.targetPos);
        setLiftState();
        SmartDashboard.putNumber("lift height", Hardware.Outtake.Lift.liftEncoder.get());
    }
}

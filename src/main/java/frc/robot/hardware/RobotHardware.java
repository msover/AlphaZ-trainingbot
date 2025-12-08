package frc.robot.hardware;

import frc.robot.hardware.wrappers.AlphaTalonFX;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.intake.Pivot;
import frc.robot.subsystems.outtake.Lift;
import frc.robot.subsystems.outtake.Suction;

public class RobotHardware {
    public AlphaTalonFX activeIntakeMotor;
    public AlphaTalonFX leftStorageMotor;
    public AlphaTalonFX rightStorageMotor;
    public AlphaTalonFX pivotMotor;

    public AlphaTalonFX leftLiftMotor;
    public AlphaTalonFX rightLiftMotor;
    public AlphaTalonFX suctionMotor;

    public SwerveDrive drive;
    public Intake intake;
    public Outtake outtake;

    private static RobotHardware instance = null;
    //private boolean enabled;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        //instance.enabled = true;
        return instance;
    }

    public void initializeHardware() {
        activeIntakeMotor = new AlphaTalonFX(51, "canivore", true);
        leftStorageMotor = new AlphaTalonFX(61, "canivore", true);
        rightStorageMotor = new AlphaTalonFX(62, "canivore", false);
        pivotMotor = new AlphaTalonFX(52, "canivore", true, Pivot.ks, Pivot.kv, Pivot.ka, Pivot.kp, Pivot.ki, Pivot.kd, Pivot.vel, Pivot.acc, Pivot.jerk);

        leftLiftMotor = new AlphaTalonFX(27, "canivore", true, Lift.ks, Lift.kv, Lift.ka, Lift.kp, Lift.ki, Lift.kd, Lift.vel, Lift.acc, Lift.jerk);
        rightLiftMotor = new AlphaTalonFX(17, "canivore", false, Lift.ks, Lift.kv, Lift.ka, Lift.kp, Lift.ki, Lift.kd, Lift.vel, Lift.acc, Lift.jerk);
        suctionMotor = new AlphaTalonFX(37, "canivore", true, Suction.ks, Suction.kv, Suction.ka, Suction.kp, Suction.ki, Suction.kd, Suction.vel, Suction.acc, Suction.jerk);

        drive = new SwerveDrive();
        intake = new Intake();
        outtake = new Outtake();
    }

    public void initializeActuators() {
        drive.initialize();
        intake.initialize();
        outtake.initialize();
    }

    public void loop() {
        drive.loop();
        intake.loop();
        //outtake.loop();
    }
}
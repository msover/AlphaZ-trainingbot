package frc.robot.hardware;

import frc.robot.hardware.wrappers.AlphaTalonFX;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.intake.Pivot;
import frc.robot.subsystems.outtake.Lift;
import frc.robot.subsystems.outtake.Suction;

public class RobotHardware {
    public AlphaTalonFX activeIntakeMotor = new AlphaTalonFX(51, "canivore", true);
    public AlphaTalonFX leftStorageMotor = new AlphaTalonFX(61, "canivore", true);
    public AlphaTalonFX rightStorageMotor = new AlphaTalonFX(62, "canivore", false);
    public AlphaTalonFX pivotMotor = new AlphaTalonFX(52, "canivore", true, Pivot.ks, Pivot.kv, Pivot.ka, Pivot.kp, Pivot.ki, Pivot.kd, Pivot.vel, Pivot.acc, Pivot.jerk);

    public AlphaTalonFX leftLiftMotor = new AlphaTalonFX(17, "canivore", false, Lift.ks, Lift.kv, Lift.ka, Lift.kp, Lift.ki, Lift.kd, Lift.vel, Lift.acc, Lift.jerk);
    public AlphaTalonFX rightLiftMotor = new AlphaTalonFX(27, "canivore", true, Lift.ks, Lift.kv, Lift.ka, Lift.kp, Lift.ki, Lift.kd, Lift.vel, Lift.acc, Lift.jerk);
    public AlphaTalonFX suctionMotor = new AlphaTalonFX(37, "canivore", true, Suction.ks, Suction.kv, Suction.ka, Suction.kp, Suction.ki, Suction.kd, Suction.vel, Suction.acc, Suction.jerk);

    public SwerveDrive drive = new SwerveDrive();
    public Intake intake = new Intake();
    public Outtake outtake = new Outtake();

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
        // activeIntakeMotor = new TalonFX(51, "canivore");
        // pivotMotor = new TalonFX(52, "canivore");
        // leftStorageMotor = new TalonFX(61, "canivore");
        // rightStorageMotor = new TalonFX(62, "canivore");

        // leftLiftMotor = new TalonFX(17, "canivore");
        // rightLiftMotor = new TalonFX(27, "canivore");
        // suctionMotor = new TalonFX(37, "canivore");
    }

    public void initializeActuators() {
        drive.initialize();
        intake.initialize();
        outtake.initialize();
    }

    public void loop() {
        drive.loop();
        intake.loop();
        outtake.loop();
    }
}
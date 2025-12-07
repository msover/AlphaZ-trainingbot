package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.hardware.RobotHardware;
import frc.robot.subsystems.Subsystem;

public class Pivot implements Subsystem {

    private final RobotHardware robot = RobotHardware.getInstance();

    public enum PivotState {
        UP,
        DOWN
    }

    public PivotState pivotState = PivotState.UP;

    public static double UP = 7;
    public static double DOWN = -0.5;

    public static double ks = 0;
    public static double kv = 0;
    public static double ka = 0;
    public static double kp = 4;
    public static double ki = 0;
    public static double kd = 0.1;
    public static double kf = 0.31;

    public static double vel = 40;
    public static double acc = 100;
    public static double jerk = 1000;

    public static double target = 0;

    boolean start = false;

    @Override
    public void initialize() {
        robot.pivotMotor.setEncoderPosition(11);
        initTelemetry();
    }

    @Override
    public void loop() {
        robot.pivotMotor.setFeedForward(kf*Math.cos(Math.toRadians((90/8.35)*robot.pivotMotor.getPosition())));

        //TODO comment this after done tuning
        // robot.pivotMotor.setMotionMagicCoefficients(ks, kv, ka, kp, ki, kd, vel, acc, jerk);

        // robot.pivotMotor.updateMotionMagicCoefficients();

        // robot.pivotMotor.applyMotorConfig();

        //robot.pivotMotor.setMotionMagicPosition(target);

        // if (!start) {
        //     setState(pivotState);
        //     start = true;
        // }

        updateState();
        updateTelemetry();
    }

    public void setState(PivotState pivotState) {
        this.pivotState = pivotState;
        switch (pivotState) {
            case UP:
                setPosition(UP);
                break;
            case DOWN:
                setPosition(DOWN);
                break;
        }
    }

    public void updateState() {
        switch (pivotState) {
            case UP:
                break;
            case DOWN:
                break;
        }
    }
    
    public void setPosition(double position) {
        target = position;
        robot.pivotMotor.setMotionMagicPosition(position);
    }

    public void initTelemetry() {
        SmartDashboard.putNumber("pivot ks", ks);
        SmartDashboard.putNumber("pivot kv", kv);
        SmartDashboard.putNumber("pivot ka", ka);
        SmartDashboard.putNumber("pivot kp", kp);
        SmartDashboard.putNumber("pivot ki", ki);
        SmartDashboard.putNumber("pivot kd", kd);
        SmartDashboard.putNumber("pivot kf", kf);
        SmartDashboard.putNumber("pivot vel", vel);
        SmartDashboard.putNumber("pivot acc", acc);
        SmartDashboard.putNumber("pivot jerk", jerk);
    }

    public void updateTelemetry() {
        SmartDashboard.putNumber("pivot position", robot.pivotMotor.getPosition());
        SmartDashboard.putNumber("pivot target", target);
        SmartDashboard.putNumber("pivot error", robot.pivotMotor.getPositionError());
        SmartDashboard.putNumber("pivot supply current", robot.pivotMotor.getSupplyCurrent());
        SmartDashboard.putNumber("pivot stator current", robot.pivotMotor.getStatorCurrent());
        SmartDashboard.putNumber("pivot voltage", robot.pivotMotor.getVoltage());
        SmartDashboard.putNumber("pivot supply voltage", robot.pivotMotor.getSupplyVoltage());
        ks = SmartDashboard.getNumber("pivot ks", ks);
        kv = SmartDashboard.getNumber("pivot kv", kv);
        ka = SmartDashboard.getNumber("pivot ka", ka);
        kp = SmartDashboard.getNumber("pivot kp", kp);
        ki = SmartDashboard.getNumber("pivot ki", ki);
        kd = SmartDashboard.getNumber("pivot kd", kd);
        kf = SmartDashboard.getNumber("pivot kf", kf);
        vel = SmartDashboard.getNumber("pivot vel", vel);
        acc = SmartDashboard.getNumber("pivot acc", acc);
        jerk = SmartDashboard.getNumber("pivot jerk", jerk);
    }
}

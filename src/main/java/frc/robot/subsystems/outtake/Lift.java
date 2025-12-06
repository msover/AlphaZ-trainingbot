package frc.robot.subsystems.outtake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.hardware.RobotHardware;
import frc.robot.subsystems.Subsystem;

public class Lift implements Subsystem {

    private final RobotHardware robot = RobotHardware.getInstance();

    public enum LiftState {
        IN,
        PASSTHROUGH,
        TRANSFER,
        SCORE_LOW
    }

    public LiftState liftState = LiftState.IN;

    public static double IN = 0;
    public static double PASSTHROUGH = 3;
    public static double TRANSFER = 0.5;
    public static double SCORE_LOW_LEFT = 8.5;
    public static double SCORE_LOW_RIGHT = 5.5;

    public static double ks = 0;
    public static double kv = 0.12;
    public static double ka = 0.01;
    public static double kp = 13.5;
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 0.25;

    public static double vel = 35;
    public static double acc = 150;
    public static double jerk = 1000;

    public static double errorThreshold = 0.1;

    public static double targetLeft = 0;
    public static double targetRight = 0;

    @Override
    public void initialize() {
        robot.leftLiftMotor.setEncoderPosition(0);
        robot.rightLiftMotor.setEncoderPosition(0);
        initTelemetry();
    }

    @Override
    public void loop() {
        robot.leftLiftMotor.setFeedForward(kf);
        robot.rightLiftMotor.setFeedForward(kf);

        //TODO comment this after done tuning
        robot.leftLiftMotor.setMotionMagicCoefficients(ks, kv, ka, kp, ki, kd, vel, acc, jerk);
        robot.rightLiftMotor.setMotionMagicCoefficients(ks, kv, ka, kp, ki, kd, vel, acc, jerk);

        robot.leftLiftMotor.updateMotionMagicCoefficients();
        robot.rightLiftMotor.updateMotionMagicCoefficients();

        robot.leftLiftMotor.applyMotorConfig();
        robot.rightLiftMotor.applyMotorConfig();

        updateState();
        updateTelemetry();
    }
    
    public void setState(LiftState liftState) {
        this.liftState = liftState;
        switch (liftState) {
            case IN:
                setPosition(IN);
                break;
            case PASSTHROUGH:
                setPosition(PASSTHROUGH);
                break;
            case TRANSFER:
                setPosition(TRANSFER);
                break;
            case SCORE_LOW:
                setLeftPosition(SCORE_LOW_LEFT);
                setRightPosition(SCORE_LOW_RIGHT);
                break;
        }
    }

    public void updateState() {
        switch (liftState) {
            case IN:
                break;
            case PASSTHROUGH:
                break;
            case TRANSFER:
                break;
            case SCORE_LOW:
                break;
        }
    }

    public void setLeftPosition(double position) {
        targetLeft = position;
        robot.leftLiftMotor.setMotionMagicPosition(position);
    }

    public void setRightPosition(double position) {
        targetRight = position;
        robot.rightLiftMotor.setMotionMagicPosition(position);
    }

    public void setPosition(double position) {
        targetLeft = position;
        targetRight = position;
        robot.leftLiftMotor.setMotionMagicPosition(position);
        robot.rightLiftMotor.setMotionMagicPosition(position);
    }

    public double getLeftPosition() {
        return robot.leftLiftMotor.getPosition();
    }

    public double getRightPosition() {
        return robot.rightLiftMotor.getPosition();
    }

    public double getLeftPositionError() {
        return robot.leftLiftMotor.getPositionError();
    }

    public double getRightPositionError() {
        return robot.rightLiftMotor.getPositionError();
    }

    public boolean inPosition(double error) {
        return getLeftPositionError() < error && getRightPositionError() < error;
    }

    public void initTelemetry() {
        SmartDashboard.putNumber("lift ks", ks);
        SmartDashboard.putNumber("lift kv", kv);
        SmartDashboard.putNumber("lift ka", ka);
        SmartDashboard.putNumber("lift kp", kp);
        SmartDashboard.putNumber("lift ki", ki);
        SmartDashboard.putNumber("lift kd", kd);
        SmartDashboard.putNumber("lift kf", kf);
        SmartDashboard.putNumber("lift vel", vel);
        SmartDashboard.putNumber("lift acc", acc);
        SmartDashboard.putNumber("lift jerk", jerk);
    }

    public void updateTelemetry() {
        SmartDashboard.putNumber("left lift position", getLeftPosition());
        SmartDashboard.putNumber("left lift target", targetLeft);
        SmartDashboard.putNumber("right lift position", getRightPosition());
        SmartDashboard.putNumber("right lift target", targetRight);
        SmartDashboard.putNumber("left lift error", getLeftPositionError());
        SmartDashboard.putNumber("right lift error", getRightPositionError());
        SmartDashboard.putNumber("left lift supply current", robot.leftLiftMotor.getSupplyCurrent());
        SmartDashboard.putNumber("right lift supply current", robot.rightLiftMotor.getSupplyCurrent());
        SmartDashboard.putNumber("left lift stator current", robot.leftLiftMotor.getStatorCurrent());
        SmartDashboard.putNumber("right lift stator current", robot.rightLiftMotor.getStatorCurrent());
        SmartDashboard.putNumber("left lift voltage", robot.leftLiftMotor.getVoltage());
        SmartDashboard.putNumber("right lift voltage", robot.rightLiftMotor.getVoltage());
        SmartDashboard.putNumber("left lift supply voltage", robot.leftLiftMotor.getSupplyVoltage());
        SmartDashboard.putNumber("right lift supply voltage", robot.rightLiftMotor.getSupplyVoltage());
        ks = SmartDashboard.getNumber("lift ks", ks);
        kv = SmartDashboard.getNumber("lift kv", kv);
        ka = SmartDashboard.getNumber("lift ka", ka);
        kp = SmartDashboard.getNumber("lift kp", kp);
        ki = SmartDashboard.getNumber("lift ki", ki);
        kd = SmartDashboard.getNumber("lift kd", kd);
        kf = SmartDashboard.getNumber("lift kf", kf);
        vel = SmartDashboard.getNumber("lift vel", vel);
        acc = SmartDashboard.getNumber("lift acc", acc);
        jerk = SmartDashboard.getNumber("lift jerk", jerk);
    }
}

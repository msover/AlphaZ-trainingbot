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

    public LiftState liftState = LiftState.PASSTHROUGH;

    public static double IN = 0;
    public static double PASSTHROUGH = 3;
    public static double TRANSFER = 0.83;
    public static double SCORE_LOW_LEFT = 5.5;
    public static double SCORE_LOW_RIGHT = 5.5;

    public static double ks = 0;
    public static double kv = 0;
    public static double ka = 0;
    public static double kp = 5.5; //5.5
    public static double ki = 0;
    public static double kd = 0.2;//0.2
    public static double kf = 0.4; //0.4
    public static double kf_angle = 0.85; //0.9

    public static double angle = 0;

    public static double vel = 10; //10
    public static double acc = 5; //2
    public static double jerk = 20; //5

    public static double errorThreshold = 0.2;
    public static double timeThresholdTransfer = 1;

    public static double targetLeft = PASSTHROUGH;
    public static double targetRight = PASSTHROUGH;

    public static boolean tuning = false;


    @Override
    public void initialize() {
        robot.leftLiftMotor.setEncoderPosition(0);
        robot.rightLiftMotor.setEncoderPosition(0);

        initTelemetry();
    }

    @Override
    public void loop() {

        robot.leftLiftMotor.setFeedForward(kf + kf_angle*Math.sin(Math.toRadians(getAngle())));
        robot.rightLiftMotor.setFeedForward(kf - kf_angle*Math.sin(Math.toRadians(getAngle())));

        //TODO comment this after done tuning
        if (tuning) {
            robot.leftLiftMotor.setMotionMagicCoefficients(ks, kv, ka, kp, ki, kd, vel, acc, jerk);
            robot.rightLiftMotor.setMotionMagicCoefficients(ks, kv, ka, kp, ki, kd, vel, acc, jerk);

            robot.leftLiftMotor.updateMotionMagicCoefficients();
            robot.rightLiftMotor.updateMotionMagicCoefficients();

            robot.leftLiftMotor.applyMotorConfig();
            robot.rightLiftMotor.applyMotorConfig();
        }

        setLeftPosition(targetLeft);
        setRightPosition(targetRight);

        updateState();
        updateTelemetry();
    }
    
    public void setState(LiftState liftState) {
        this.liftState = liftState;
        switch (liftState) {
            case IN:
                targetLeft = IN;
                targetRight = IN;
                break;
            case PASSTHROUGH:
                targetLeft = PASSTHROUGH;
                targetRight = PASSTHROUGH;
                break;
            case TRANSFER:
                targetLeft = TRANSFER;
                targetRight = TRANSFER;
                break;
            case SCORE_LOW:
                targetLeft = SCORE_LOW_LEFT;
                targetRight = SCORE_LOW_RIGHT;
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
        robot.leftLiftMotor.setMotionMagicPosition(position);
    }

    public void setRightPosition(double position) {
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
        //return robot.leftLiftMotor.getPositionError();
        return Math.abs(robot.leftLiftMotor.getPosition()-targetLeft);
    }

    public double getRightPositionError() {
        //return robot.rightLiftMotor.getPositionError();
        return Math.abs(robot.rightLiftMotor.getPosition()-targetRight);
    }

    public boolean inPosition(double error) {
        return getLeftPositionError() < error && getRightPositionError() < error;
    }

    public double getAngle() {
        return 90/2.321*(getLeftPosition() - getRightPosition());
    }

    public void initTelemetry() {
        SmartDashboard.putNumber("lift ks", ks);
        SmartDashboard.putNumber("lift kv", kv);
        SmartDashboard.putNumber("lift ka", ka);
        SmartDashboard.putNumber("lift kp", kp);
        SmartDashboard.putNumber("lift ki", ki);
        SmartDashboard.putNumber("lift kd", kd);
        SmartDashboard.putNumber("lift kf", kf);
        SmartDashboard.putNumber("lift kf_angle", kf_angle);
        SmartDashboard.putNumber("lift vel", vel);
        SmartDashboard.putNumber("lift acc", acc);
        SmartDashboard.putNumber("lift jerk", jerk);
        SmartDashboard.putNumber("lift target left", targetLeft);
        SmartDashboard.putNumber("lift target right", targetRight);
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
        kf_angle = SmartDashboard.getNumber("lift kf_angle", kf_angle);
        vel = SmartDashboard.getNumber("lift vel", vel);
        acc = SmartDashboard.getNumber("lift acc", acc);
        jerk = SmartDashboard.getNumber("lift jerk", jerk);
        // targetLeft = SmartDashboard.getNumber("lift target left", targetLeft);
        // targetRight = SmartDashboard.getNumber("lift target right", targetRight);
    }
}

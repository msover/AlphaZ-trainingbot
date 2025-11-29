package frc.robot.subsystems.outtake.lift;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.constants.Constants;
import frc.robot.utils.hardware.Hardware;

public class Lift {
    private enum ArmState {
        IN,
        TRANSFER,
        SAFE,
        SCORE1,
        SCORE2,
        MANUAL,
    }

    private enum liftFSM {
        COLLECT
    }

    private PS5Controller gamepad;
    private static Lift instance;

    public static Lift getInstace() {
        if (instance == null) {
            instance = new Lift();
        }
        return instance;
    }

    double leftRotation = 0;
    double rightRotation = 0;
    
    private TalonFX left = Hardware.Outtake.Lift.leftMotor;
    private TalonFX right = Hardware.Outtake.Lift.rightMotor;
    private MotionMagicVoltage request;

    public void setMotorConfig(TalonFX motor, double ks, double kv, double ka, double kp, double ki, double kd,
            int magicVel, int magicAcc, int magicJerk) {
        var motorConfig = new TalonFXConfiguration();
        motorConfig.Slot0.kS = ks;
        motorConfig.Slot0.kV = kv;
        motorConfig.Slot0.kA = ka;
        motorConfig.Slot0.kP = kp;
        motorConfig.Slot0.kI = ki;
        motorConfig.Slot0.kD = kd;

        var magicConfig = motorConfig.MotionMagic;
        magicConfig.MotionMagicCruiseVelocity = magicVel;
        magicConfig.MotionMagicAcceleration = magicAcc;
        magicConfig.MotionMagicJerk = magicJerk;

        motor.getConfigurator().apply(motorConfig);
    }

    public void setMotorConfig(TalonFX motor, int magicVel, int magicAcc, int magicJerk) {
        var motorConfig = new TalonFXConfiguration();

        var magicConfig = motorConfig.MotionMagic;
        magicConfig.MotionMagicCruiseVelocity = magicVel;
        magicConfig.MotionMagicAcceleration = magicAcc;
        magicConfig.MotionMagicJerk = magicJerk;

        motor.getConfigurator().apply(motorConfig);
    }

    public void init(PS5Controller gamepad) {
        this.gamepad = gamepad;
        setMotorConfig(
                left,
                Constants.Outtake.Lift.ks,
                Constants.Outtake.Lift.kv,
                Constants.Outtake.Lift.ka,
                Constants.Outtake.Lift.kp,
                Constants.Outtake.Lift.ki,
                Constants.Outtake.Lift.kd,
                Constants.Outtake.Lift.cruiseVel,
                Constants.Outtake.Lift.acc,
                Constants.Outtake.Lift.jerk);

        setMotorConfig(
                right,
                Constants.Outtake.Lift.ks,
                Constants.Outtake.Lift.kv,
                Constants.Outtake.Lift.ka,
                Constants.Outtake.Lift.kp,
                Constants.Outtake.Lift.ki,
                Constants.Outtake.Lift.kd,
                Constants.Outtake.Lift.cruiseVel,
                Constants.Outtake.Lift.acc,
                Constants.Outtake.Lift.jerk);
    }



    private double getMotorTicks(TalonFX motor) {
        return motor.getPosition().getValueAsDouble();
    }

    private void setTargets(ArmState armState) {
        switch (armState) {
            case IN:
                leftRotation = Constants.Outtake.Lift.LEFT_IN;
                rightRotation = Constants.Outtake.Lift.RIGHT_IN;
                break;
            case TRANSFER:
                leftRotation = Constants.Outtake.Lift.LEFT_TRANSFER;
                rightRotation = Constants.Outtake.Lift.RIGHT_TRANSFER;
                break;
            case SAFE:
                leftRotation = Constants.Outtake.Lift.LEFT_SAFE;
                rightRotation = Constants.Outtake.Lift.RIGHT_SAFE;
                break;
            case SCORE1:
                leftRotation = Constants.Outtake.Lift.LEFT_SCORE1;
                rightRotation = Constants.Outtake.Lift.RIGHT_SCORE1;
                break;
            case SCORE2:
                leftRotation = Constants.Outtake.Lift.LEFT_SCORE2;
                rightRotation = Constants.Outtake.Lift.RIGHT_SCORE2;
                break;
            case MANUAL:
                leftRotation = Constants.Outtake.Lift.LEFT_SCORE2;
                rightRotation = Constants.Outtake.Lift.RIGHT_SCORE2;
                break;
        }

    }
    public void initPID() {
        setMotorConfig(
                left,
                Constants.Outtake.Lift.ks,
                Constants.Outtake.Lift.kv,
                Constants.Outtake.Lift.ka,
                Constants.Outtake.Lift.kp,
                Constants.Outtake.Lift.ki,
                Constants.Outtake.Lift.kd,
                Constants.Outtake.Lift.cruiseVel,
                Constants.Outtake.Lift.acc,
                Constants.Outtake.Lift.jerk
        );

        setMotorConfig(
                right,
                Constants.Outtake.Lift.ks,
                Constants.Outtake.Lift.kv,
                Constants.Outtake.Lift.ka,
                Constants.Outtake.Lift.kp,
                Constants.Outtake.Lift.ki,
                Constants.Outtake.Lift.kd,
                Constants.Outtake.Lift.cruiseVel,
                Constants.Outtake.Lift.acc,
                Constants.Outtake.Lift.jerk
        );
        
        request = new MotionMagicVoltage(0);
    }
    private void updatePID() {
        init(gamepad);
        request.withFeedForward(Constants.Outtake.Lift.kf);
        left.setControl(request.withPosition(Constants.Outtake.Lift.TARGET_ROT));
        right.setControl(request.withPosition(-Constants.Outtake.Lift.TARGET_ROT));
    }

    private void updateHardware(ArmState armState) {
        switch (armState) {
            case SAFE:
                break;
            case TRANSFER:
                break;
            case SCORE1:
                break;
        }
    }

    public void update() {

        SmartDashboard.putNumber("left ", getMotorTicks(left));
        SmartDashboard.putNumber("right ", -getMotorTicks(right));
        ///SmartDashboard.putNumber("right - left ", -getMotorTicks(right) - getMotorTicks(left));
        SmartDashboard.putNumber("target rot", Constants.Outtake.Lift.TARGET_ROT);
        SmartDashboard.putNumber("kf", Constants.Outtake.Lift.kf);
        ///Shuffleboard.getTab("PID").add("kfffff", Constants.Outtake.Lift.kf);
        SmartDashboard.putNumber("ks", Constants.Outtake.Lift.ks);
        SmartDashboard.putNumber("kv", Constants.Outtake.Lift.kv);
        SmartDashboard.putNumber("ka", Constants.Outtake.Lift.ka);
        SmartDashboard.putNumber("kp", Constants.Outtake.Lift.kp);
        SmartDashboard.putNumber("ki", Constants.Outtake.Lift.ki);
        SmartDashboard.putNumber("kd", Constants.Outtake.Lift.kd);
        SmartDashboard.putNumber("cruiseVel", Constants.Outtake.Lift.cruiseVel);
        SmartDashboard.putNumber("accell", Constants.Outtake.Lift.acc);
        SmartDashboard.putNumber("jerk", Constants.Outtake.Lift.jerk);
        

        updatePID();

    }

}

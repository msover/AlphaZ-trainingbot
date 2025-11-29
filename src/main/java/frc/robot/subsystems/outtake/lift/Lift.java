package frc.robot.subsystems.outtake.lift;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

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

    private static Lift instance;

    public static Lift getInstace() {
        if (instance == null) {
            instance = new Lift();
        }
        return instance;
    }

    double leftRotation = 0;
    double rightRotation = 0;

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

    public void init() {
        setMotorConfig(left, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        setMotorConfig(right, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }

    private TalonFX left = Hardware.Outtake.Lift.leftMotor;

    private TalonFX right = Hardware.Outtake.Lift.rightMotor;

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

    private void updatePID() {
        final MotionMagicVoltage request = new MotionMagicVoltage(0);
        left.setControl(request.withPosition(leftRotation));
        right.setControl(request.withPosition(rightRotation));
    }

    private void updateHardware()
    {
        
    }

    public void update() {

        SmartDashboard.putNumber("left ", getMotorTicks(left));
        SmartDashboard.putNumber("right ", -getMotorTicks(right));
        SmartDashboard.putNumber("right - left ", -getMotorTicks(right) - getMotorTicks(left));

        updatePID();

    }

}

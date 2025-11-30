package frc.robot.subsystems.outtake.lift;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.PS5Controller;
import frc.robot.utils.constants.Constants;
import frc.robot.utils.hardware.Hardware;

public class Lift {
    private enum ArmState {
        IDLE,
        TRANSFER,
        SUGATIV_SUGE,
        SAFE,
        SCORE1,
        SCORE2,
    }

    private PS5Controller gamepad;
    private static Lift instance;

    public static Lift getInstace() {
        if (instance == null) {
            instance = new Lift();
        }
        return instance;
    }

    private ArmState armState = ArmState.SAFE;
    private ArmState scheduledState = ArmState.SAFE;

    private TalonFX left = Hardware.Outtake.Lift.leftMotor;
    private TalonFX right = Hardware.Outtake.Lift.rightMotor;
    private TalonFX sugativ = Hardware.Outtake.Lift.sugativMotor;
    private MotionMagicVoltage request;

    public void setMotorConfig(TalonFX motor, double ks, double kv, double ka, double kp, double ki, double kd,
            int magicVel, int magicAcc, int magicJerk, boolean reversed) {
        var motorConfig = new TalonFXConfiguration();

        motorConfig.Voltage.PeakForwardVoltage = 12;
        motorConfig.Voltage.PeakReverseVoltage = -12;

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

        if (reversed) {
            motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        } else {
            motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }

        //TODO current limits
        motorConfig.CurrentLimits.withSupplyCurrentLowerLimit(Amps.of(30));
        motorConfig.CurrentLimits.withSupplyCurrentLimit(Amps.of(40));
        motorConfig.CurrentLimits.withSupplyCurrentLowerTime(Seconds.of(0.2));
        motorConfig.CurrentLimits.withSupplyCurrentLimitEnable(true);
        motorConfig.CurrentLimits.withStatorCurrentLimit(Amps.of(80));
        motorConfig.CurrentLimits.withStatorCurrentLimitEnable(true);

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
        // SmartDashboard.putNumber("target rot left", Constants.Outtake.Lift.TARGET_ROT_LEFT);
        // SmartDashboard.putNumber("target rot right", Constants.Outtake.Lift.TARGET_ROT_RIGHT);
        // SmartDashboard.putNumber("kf", Constants.Outtake.Lift.kf);
        // SmartDashboard.putNumber("ks", Constants.Outtake.Lift.ks);
        // SmartDashboard.putNumber("kv", Constants.Outtake.Lift.kv);
        // SmartDashboard.putNumber("ka", Constants.Outtake.Lift.ka);
        // SmartDashboard.putNumber("kp", Constants.Outtake.Lift.kp);
        // SmartDashboard.putNumber("ki", Constants.Outtake.Lift.ki);
        // SmartDashboard.putNumber("kd", Constants.Outtake.Lift.kd);
        // SmartDashboard.putNumber("cruiseVel", Constants.Outtake.Lift.cruiseVel);
        // SmartDashboard.putNumber("accell", Constants.Outtake.Lift.acc);
        // SmartDashboard.putNumber("jerk", Constants.Outtake.Lift.jerk);
        // SmartDashboard.putBoolean("TRAP LOOP", Constants.Outtake.Lift.trapLoop);
        this.gamepad = gamepad;
        setMotorConfig(
                left,
                Constants.Outtake.PID.ks,
                Constants.Outtake.PID.kv,
                Constants.Outtake.PID.ka,
                Constants.Outtake.PID.kp,
                Constants.Outtake.PID.ki,
                Constants.Outtake.PID.kd,
                Constants.Outtake.PID.cruiseVel,
                Constants.Outtake.PID.acc,
                Constants.Outtake.PID.jerk,
                false);

        setMotorConfig(
                right,
                Constants.Outtake.PID.ks,
                Constants.Outtake.PID.kv,
                Constants.Outtake.PID.ka,
                Constants.Outtake.PID.kp,
                Constants.Outtake.PID.ki,
                Constants.Outtake.PID.kd,
                Constants.Outtake.PID.cruiseVel,
                Constants.Outtake.PID.acc,
                Constants.Outtake.PID.jerk,
                true);
        
        

                left.setPosition(0);
                right.setPosition(0);
                
                request = new MotionMagicVoltage(0);
                request.withFeedForward(Constants.Outtake.PID.kf);
                left.setControl(request.withPosition(Constants.Outtake.Lift.SAFE).withSlot(0));
                right.setControl(request.withPosition(Constants.Outtake.Lift.SAFE).withSlot(0));
    }

    public void update() {
        switch(armState) {
            case IDLE:
                if(gamepad.getCrossButtonPressed()) {
                    armState = ArmState.TRANSFER;
                    scheduledState = ArmState.SCORE1;
                }
                if(gamepad.getSquareButtonPressed()) {
                    armState = ArmState.TRANSFER;
                    scheduledState = ArmState.SCORE2;
                }
                left.setControl(request.withPosition(Constants.Outtake.Lift.SAFE).withSlot(0));
                right.setControl(request.withPosition(Constants.Outtake.Lift.SAFE).withSlot(0));
                break;
            

            case TRANSFER:
                if(Math.abs(left.getPosition().getValueAsDouble() - Constants.Outtake.Lift.TRANSFER) < Constants.Outtake.PID.PERMISSIVE_ERROR) {
                    armState = ArmState.SUGATIV_SUGE;
                }
                left.setControl(request.withPosition(Constants.Outtake.Lift.TRANSFER).withSlot(0));
                right.setControl(request.withPosition(Constants.Outtake.Lift.TRANSFER).withSlot(0));
                break;

            case SUGATIV_SUGE:
                if(Math.abs(sugativ.getPosition().getValueAsDouble() - Constants.Outtake.Lift.SUGATIV_SUGE) < Constants.Outtake.PID.PERMISSIVE_ERROR) {
                    armState = ArmState.SAFE;
                }
                sugativ.setControl(request.withPosition(Constants.Outtake.Lift.SUGATIV_SUGE).withSlot(0));
                break;

            case SAFE:
                if(Math.abs(sugativ.getPosition().getValueAsDouble() - Constants.Outtake.Lift.SAFE) < Constants.Outtake.PID.PERMISSIVE_ERROR) {
                    armState = scheduledState;
                }
                left.setControl(request.withPosition(Constants.Outtake.Lift.SAFE).withSlot(0));
                right.setControl(request.withPosition(Constants.Outtake.Lift.SAFE).withSlot(0));
                break;

            case SCORE1:
                left.setControl(request.withPosition(Constants.Outtake.Lift.LEFT_SCORE1).withSlot(0));
                right.setControl(request.withPosition(Constants.Outtake.Lift.RIGHT_SCORE1).withSlot(0));
                if(gamepad.getCrossButtonPressed()) {
                    scheduledState = ArmState.IDLE;
                }
                break;
            
            case SCORE2:
                left.setControl(request.withPosition(Constants.Outtake.Lift.LEFT_SCORE2).withSlot(0));
                right.setControl(request.withPosition(Constants.Outtake.Lift.RIGHT_SCORE2).withSlot(0));
                if(gamepad.getSquareButtonPressed()) {
                    scheduledState = ArmState.IDLE;
                }
                break;
        }
        

        ///SmartDashboard.putNumber("left ", getMotorTicks(left));
        ///SmartDashboard.putNumber("right ", getMotorTicks(right));
        ///SmartDashboard.putNumber("right - left ", -getMotorTicks(right) - getMotorTicks(left));
        
        // Constants.Outtake.Lift.TARGET_ROT_LEFT = SmartDashboard.getNumber("target rot left", Constants.Outtake.Lift.TARGET_ROT_LEFT);
        // Constants.Outtake.Lift.TARGET_ROT_RIGHT = SmartDashboard.getNumber("target rot right", Constants.Outtake.Lift.TARGET_ROT_RIGHT);
        // Constants.Outtake.Lift.kf = SmartDashboard.getNumber("kf", Constants.Outtake.Lift.kf);
        // Constants.Outtake.Lift.ks = SmartDashboard.getNumber("ks", Constants.Outtake.Lift.ks);
        // Constants.Outtake.Lift.kv = SmartDashboard.getNumber("kv", Constants.Outtake.Lift.kv);
        // Constants.Outtake.Lift.ka = SmartDashboard.getNumber("ka", Constants.Outtake.Lift.ka);
        // Constants.Outtake.Lift.kp = SmartDashboard.getNumber("kp", Constants.Outtake.Lift.kp);
        // Constants.Outtake.Lift.ki = SmartDashboard.getNumber("ki", Constants.Outtake.Lift.ki);
        // Constants.Outtake.Lift.kd = SmartDashboard.getNumber("kd", Constants.Outtake.Lift.kd);
        // Constants.Outtake.Lift.cruiseVel = (int)SmartDashboard.getNumber("cruiseVel", Constants.Outtake.Lift.cruiseVel);
        // Constants.Outtake.Lift.acc = (int)SmartDashboard.getNumber("accell", Constants.Outtake.Lift.acc);
        // Constants.Outtake.Lift.jerk = (int)SmartDashboard.getNumber("jerk", Constants.Outtake.Lift.jerk);
        // Constants.Outtake.Lift.trapLoop = SmartDashboard.getBoolean("TRAP LOOP",Constants.Outtake.Lift.trapLoop);
    }

}

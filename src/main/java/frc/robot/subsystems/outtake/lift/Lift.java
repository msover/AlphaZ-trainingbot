package frc.robot.subsystems.outtake.lift;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.constants.Constants;
import frc.robot.utils.hardware.Hardware;

public class Lift {
    private enum ArmState {
        IDLE,
        TRANSFER,
        SUGATIV_SUGE,
        SAFE,
        SCORE,
        SCORE1_DISPARITY,
        SCORE2_DISPARITY,
        SCORE3_DISPARITY,
        SUGATIV_LASA,
        SCORE1,
        SCORE2,
        SCORE3
    }

    private PS5Controller gamepad;
    private static Lift instance;

    public static Lift getInstace() {
        if (instance == null) {
            instance = new Lift();
        }
        return instance;
    }

    private ArmState armState = ArmState.IDLE;
    private ArmState scheduledState = ArmState.IDLE;

    private TalonFX left = Hardware.Outtake.Lift.leftMotor;
    private TalonFX right = Hardware.Outtake.Lift.rightMotor;
    private TalonFX sugativ = Hardware.Outtake.Lift.sugativMotor;
    private MotionMagicVoltage liftRequest;
    private MotionMagicVoltage sugativRequest;
    private long startTime;

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
        this.gamepad = gamepad;
        setMotorConfig(
                left,
                Constants.Outtake.LiftPID.ks,
                Constants.Outtake.LiftPID.kv,
                Constants.Outtake.LiftPID.ka,
                Constants.Outtake.LiftPID.kp,
                Constants.Outtake.LiftPID.ki,
                Constants.Outtake.LiftPID.kd,
                Constants.Outtake.LiftPID.cruiseVel,
                Constants.Outtake.LiftPID.acc,
                Constants.Outtake.LiftPID.jerk,
                false);

        setMotorConfig(
                right,
                Constants.Outtake.LiftPID.ks,
                Constants.Outtake.LiftPID.kv,
                Constants.Outtake.LiftPID.ka,
                Constants.Outtake.LiftPID.kp,
                Constants.Outtake.LiftPID.ki,
                Constants.Outtake.LiftPID.kd,
                Constants.Outtake.LiftPID.cruiseVel,
                Constants.Outtake.LiftPID.acc,
                Constants.Outtake.LiftPID.jerk,
                true);

        setMotorConfig(
            sugativ,
            Constants.Outtake.SugativPID.ks,
            Constants.Outtake.SugativPID.kv,
            Constants.Outtake.SugativPID.ka,
            Constants.Outtake.SugativPID.kp,
            Constants.Outtake.SugativPID.ki,
            Constants.Outtake.SugativPID.kd,
            Constants.Outtake.SugativPID.cruiseVel,
            Constants.Outtake.SugativPID.acc,
            Constants.Outtake.SugativPID.jerk,
            true
        );
        
        

        left.setPosition(0);
        right.setPosition(0);

        sugativ.setPosition(0);
        
        liftRequest = new MotionMagicVoltage(0);
        liftRequest.withFeedForward(Constants.Outtake.LiftPID.kf);
        sugativRequest = new MotionMagicVoltage(0);

        startTime = System.currentTimeMillis();
        initDash();
    }

    private void fsm() {
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
                if(gamepad.getTriangleButtonPressed()) {
                    armState = ArmState.TRANSFER;
                    scheduledState = ArmState.SCORE3;
                }
                left.setControl(liftRequest.withPosition(Constants.Outtake.Lift.SAFE).withSlot(0));
                right.setControl(liftRequest.withPosition(Constants.Outtake.Lift.SAFE).withSlot(0));

                break;
            
            case TRANSFER:
                if(Math.abs(left.getPosition().getValueAsDouble() - Constants.Outtake.Lift.TRANSFER) < Constants.Outtake.LiftPID.PERMISSIVE_COLLECTION_ERROR) {
                    armState = ArmState.SUGATIV_SUGE;
                }
                left.setControl(liftRequest.withPosition(Constants.Outtake.Lift.TRANSFER).withSlot(0));
                right.setControl(liftRequest.withPosition(Constants.Outtake.Lift.TRANSFER).withSlot(0));
                break;

            case SUGATIV_SUGE:
                if(Math.abs(sugativ.getPosition().getValueAsDouble() - Constants.Outtake.Lift.SUGATIV_SUGE) < Constants.Outtake.LiftPID.PERMISSIVE_ERROR) {
                    armState = ArmState.SAFE;
                }
                sugativ.setControl(sugativRequest.withPosition(Constants.Outtake.Lift.SUGATIV_SUGE).withSlot(0));
                break;

            case SAFE:
                if(Math.abs(left.getPosition().getValueAsDouble() - Constants.Outtake.Lift.SAFE) < Constants.Outtake.LiftPID.PERMISSIVE_ERROR) {
                    armState = scheduledState;
                }
                left.setControl(liftRequest.withPosition(Constants.Outtake.Lift.SAFE).withSlot(0));
                right.setControl(liftRequest.withPosition(Constants.Outtake.Lift.SAFE).withSlot(0));
                break;

            
            case SCORE1_DISPARITY:
                if(gamepad.getCrossButtonPressed()) {
                    armState = ArmState.IDLE;
                    scheduledState = ArmState.IDLE;
                }    
                if(gamepad.getTouchpadButtonPressed()) {
                    scheduledState = armState;
                    armState = ArmState.SUGATIV_LASA;
                }
                left.setControl(liftRequest.withPosition(Constants.Outtake.Lift.SCORE1 - Constants.Outtake.Lift.SCORE1_DISPARITY).withSlot(0));
                right.setControl(liftRequest.withPosition(Constants.Outtake.Lift.SCORE1 + Constants.Outtake.Lift.SCORE1_DISPARITY).withSlot(0));
                break;
            case SCORE2_DISPARITY:
                if(gamepad.getSquareButtonPressed()) {
                    armState = ArmState.IDLE;
                    scheduledState = ArmState.IDLE;
                }    
                if(gamepad.getTouchpadButtonPressed()) {
                    scheduledState = armState;
                    armState = ArmState.SUGATIV_LASA;
                    
                }
                
            
                left.setControl(liftRequest.withPosition(Constants.Outtake.Lift.SCORE2 - Constants.Outtake.Lift.SCORE2_DISPARITY).withSlot(0));
                right.setControl(liftRequest.withPosition(Constants.Outtake.Lift.SCORE2 + Constants.Outtake.Lift.SCORE2_DISPARITY).withSlot(0));
            
                break;
            case SCORE3_DISPARITY:
                if(gamepad.getTriangleButtonPressed()) {
                    armState = ArmState.IDLE;
                    scheduledState = ArmState.IDLE;
                }    
                if(gamepad.getTouchpadButtonPressed()) {
                    scheduledState = armState;
                    armState = ArmState.SUGATIV_LASA;
                }
                    left.setControl(liftRequest.withPosition(Constants.Outtake.Lift.SCORE3 - Constants.Outtake.Lift.SCORE3_DISPARITY).withSlot(0));
                    right.setControl(liftRequest.withPosition(Constants.Outtake.Lift.SCORE3 + Constants.Outtake.Lift.SCORE3_DISPARITY).withSlot(0));
                break;

            case SUGATIV_LASA:
                if(Math.abs(sugativ.getPosition().getValueAsDouble() - Constants.Outtake.Lift.SUGATIV_LASA) < Constants.Outtake.LiftPID.PERMISSIVE_ERROR) {
                    armState = scheduledState;
                }
                sugativ.setControl(sugativRequest.withPosition(Constants.Outtake.Lift.SUGATIV_LASA).withSlot(0));
                break;


            case SCORE1:
                if(Math.abs(left.getPosition().getValueAsDouble() - Constants.Outtake.Lift.SCORE1) < Constants.Outtake.LiftPID.PERMISSIVE_ERROR) {
                    armState = ArmState.SCORE1_DISPARITY;
                    scheduledState = ArmState.SCORE1_DISPARITY;
                }
                left.setControl(liftRequest.withPosition(Constants.Outtake.Lift.SCORE1).withSlot(0));
                right.setControl(liftRequest.withPosition(Constants.Outtake.Lift.SCORE1).withSlot(0));
                break;
                
            case SCORE2:
                if(Math.abs(left.getPosition().getValueAsDouble() - Constants.Outtake.Lift.SCORE2) < Constants.Outtake.LiftPID.PERMISSIVE_ERROR) {
                    armState = ArmState.SCORE2_DISPARITY;
                    scheduledState = ArmState.SCORE2_DISPARITY;
                }
                left.setControl(liftRequest.withPosition(Constants.Outtake.Lift.SCORE2).withSlot(0));
                right.setControl(liftRequest.withPosition(Constants.Outtake.Lift.SCORE2).withSlot(0));
                break;

            case SCORE3:
                if(Math.abs(left.getPosition().getValueAsDouble() - Constants.Outtake.Lift.SCORE3) < Constants.Outtake.LiftPID.PERMISSIVE_ERROR) {
                    armState = ArmState.SCORE3_DISPARITY;
                    scheduledState = ArmState.SCORE3_DISPARITY;
                }
                left.setControl(liftRequest.withPosition(Constants.Outtake.Lift.SCORE3).withSlot(0));
                right.setControl(liftRequest.withPosition(Constants.Outtake.Lift.SCORE3).withSlot(0));
                break;
        }
    }
    private void initDash() {
        SmartDashboard.putNumber("target rot", Constants.Outtake.SugativPID.TARGET_ROT);
        SmartDashboard.putNumber("kf", Constants.Outtake.SugativPID.kf);
        SmartDashboard.putNumber("ks", Constants.Outtake.SugativPID.ks);
        SmartDashboard.putNumber("kv", Constants.Outtake.SugativPID.kv);
        SmartDashboard.putNumber("ka", Constants.Outtake.SugativPID.ka);
        SmartDashboard.putNumber("kp", Constants.Outtake.SugativPID.kp);
        SmartDashboard.putNumber("ki", Constants.Outtake.SugativPID.ki);
        SmartDashboard.putNumber("kd", Constants.Outtake.SugativPID.kd);
        SmartDashboard.putNumber("cruiseVel", Constants.Outtake.SugativPID.cruiseVel);
        SmartDashboard.putNumber("accell", Constants.Outtake.SugativPID.acc);
        SmartDashboard.putNumber("jerk", Constants.Outtake.SugativPID.jerk);
        SmartDashboard.putBoolean("TRAP LOOP", Constants.Outtake.SugativPID.trapLoop);
    }
    private void updateDash() {
        SmartDashboard.putNumber("pos", sugativ.getPosition().getValueAsDouble());
        Constants.Outtake.SugativPID.TARGET_ROT = SmartDashboard.getNumber("target rot", Constants.Outtake.SugativPID.TARGET_ROT);
        Constants.Outtake.SugativPID.kf = SmartDashboard.getNumber("kf", Constants.Outtake.SugativPID.kf);
        Constants.Outtake.SugativPID.ks = SmartDashboard.getNumber("ks", Constants.Outtake.SugativPID.ks);
        Constants.Outtake.SugativPID.kv = SmartDashboard.getNumber("kv", Constants.Outtake.SugativPID.kv);
        Constants.Outtake.SugativPID.ka = SmartDashboard.getNumber("ka", Constants.Outtake.SugativPID.ka);
        Constants.Outtake.SugativPID.kp = SmartDashboard.getNumber("kp", Constants.Outtake.SugativPID.kp);
        Constants.Outtake.SugativPID.ki = SmartDashboard.getNumber("ki", Constants.Outtake.SugativPID.ki);
        Constants.Outtake.SugativPID.kd = SmartDashboard.getNumber("kd", Constants.Outtake.SugativPID.kd);
        Constants.Outtake.SugativPID.cruiseVel = (int) SmartDashboard.getNumber("cruiseVel", Constants.Outtake.SugativPID.cruiseVel);
        Constants.Outtake.SugativPID.acc = (int) SmartDashboard.getNumber("accell", Constants.Outtake.SugativPID.acc);
        Constants.Outtake.SugativPID.jerk = (int) SmartDashboard.getNumber("jerk", Constants.Outtake.SugativPID.jerk);
        Constants.Outtake.SugativPID.trapLoop = SmartDashboard.getBoolean("TRAP LOOP",Constants.Outtake.SugativPID.trapLoop);
    }

    public void update(PS5Controller gamepad) {
        this.gamepad = gamepad;
        if(System.currentTimeMillis() - startTime > Constants.Outtake.Lift.TIMEOUT_MS) {
            fsm();
        }
        SmartDashboard.putNumber("armState", left.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("scheduled", right.getPosition().getValueAsDouble());
        
        // updateDash();
        // while(Constants.Outtake.SugativPID.trapLoop) {
        //     updateDash();        
        // }
        // sugativ.setControl(sugativRequest.withPosition(Constants.Outtake.SugativPID.TARGET_ROT).withSlot(0));
    }

}

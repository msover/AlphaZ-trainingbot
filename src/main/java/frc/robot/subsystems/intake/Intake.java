package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.constants.Constants;
import frc.robot.utils.hardware.Hardware;

public class Intake {
    private enum State {
        IDLE,
        FORWARDS,
        REVERSE
    }
    private State intakeState = State.IDLE;
    private static Intake instance;
    public static Intake getInstance() {
        if(instance == null) {
            instance = new Intake();
        }
        return instance;
    }
    private PS5Controller gamepad;

    private TalonFX left = Hardware.Intake.Storage.left;
    private TalonFX right = Hardware.Intake.Storage.right;
    private TalonFX drive = Hardware.Intake.Pivot.drive;
    private TalonFX lift = Hardware.Intake.Pivot.lift;

    private MotionMagicVoltage request;


    public void setPidConfig(TalonFX motor, double ks, double kv, double ka, double kp, double ki, double kd,
            double magicVel, double magicAcc, double magicJerk, boolean reversed) {
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
        
        motorConfig.CurrentLimits.withSupplyCurrentLowerLimit(Amps.of(20));
        motorConfig.CurrentLimits.withSupplyCurrentLimit(Amps.of(30));
        motorConfig.CurrentLimits.withSupplyCurrentLowerTime(Seconds.of(0.2));
        motorConfig.CurrentLimits.withSupplyCurrentLimitEnable(true);
        motorConfig.CurrentLimits.withStatorCurrentLimit(Amps.of(45));
        motorConfig.CurrentLimits.withStatorCurrentLimitEnable(true);

        motor.getConfigurator().apply(motorConfig);
    }

    public void setReversed(TalonFX motor, boolean reversed) {
        var motorConfig = new TalonFXConfiguration();
        if(reversed) {
            motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        } else {
            motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }
        motor.getConfigurator().apply(motorConfig);
    }

    public void init(PS5Controller gamepad) {
        this.gamepad = gamepad;

        drive.setControl(new DutyCycleOut(0));
        left.setControl(new DutyCycleOut(0));
        setReversed(left,true);
        setReversed(drive, true);
        right.setControl(new DutyCycleOut(0));
        
        request = new MotionMagicVoltage(0);

        setPidConfig(
                lift,
                Constants.Intake.PID.ks,
                Constants.Intake.PID.kv,
                Constants.Intake.PID.ka,
                Constants.Intake.PID.kp,
                Constants.Intake.PID.ki,
                Constants.Intake.PID.kd,
                Constants.Intake.PID.cruiseVel,
                Constants.Intake.PID.acc,
                Constants.Intake.PID.jerk,
                true
            );
        lift.setPosition(0.9);
        initDash();
        
        //0.7 0 power
        //0 max power
    }


    private void initDash() {
        SmartDashboard.putNumber("target rot", Constants.Intake.PID.TARGET_ROT);
        SmartDashboard.putNumber("kf", Constants.Intake.PID.kf);
        SmartDashboard.putNumber("ks", Constants.Intake.PID.ks);
        SmartDashboard.putNumber("kv", Constants.Intake.PID.kv);
        SmartDashboard.putNumber("ka", Constants.Intake.PID.ka);
        SmartDashboard.putNumber("kp", Constants.Intake.PID.kp);
        SmartDashboard.putNumber("ki", Constants.Intake.PID.ki);
        SmartDashboard.putNumber("kd", Constants.Intake.PID.kd);
        SmartDashboard.putNumber("cruiseVel", Constants.Intake.PID.cruiseVel);
        SmartDashboard.putNumber("accell", Constants.Intake.PID.acc);
        SmartDashboard.putNumber("jerk", Constants.Intake.PID.jerk);
        SmartDashboard.putBoolean("TRAP LOOP", Constants.Intake.PID.trapLoop);
    }
    private void updateDash() {
        SmartDashboard.putNumber("pos", lift.getPosition().getValueAsDouble());
        Constants.Intake.PID.TARGET_ROT = SmartDashboard.getNumber("target rot", Constants.Intake.PID.TARGET_ROT);
        Constants.Intake.PID.kf = SmartDashboard.getNumber("kf", Constants.Intake.PID.kf);
        Constants.Intake.PID.ks = SmartDashboard.getNumber("ks", Constants.Intake.PID.ks);
        Constants.Intake.PID.kv = SmartDashboard.getNumber("kv", Constants.Intake.PID.kv);
        Constants.Intake.PID.ka = SmartDashboard.getNumber("ka", Constants.Intake.PID.ka);
        Constants.Intake.PID.kp = SmartDashboard.getNumber("kp", Constants.Intake.PID.kp);
        Constants.Intake.PID.ki = SmartDashboard.getNumber("ki", Constants.Intake.PID.ki);
        Constants.Intake.PID.kd = SmartDashboard.getNumber("kd", Constants.Intake.PID.kd);
        Constants.Intake.PID.cruiseVel = SmartDashboard.getNumber("cruiseVel", Constants.Intake.PID.cruiseVel);
        Constants.Intake.PID.acc = SmartDashboard.getNumber("accell", Constants.Intake.PID.acc);
        Constants.Intake.PID.jerk = SmartDashboard.getNumber("jerk", Constants.Intake.PID.jerk);
        Constants.Intake.PID.trapLoop = SmartDashboard.getBoolean("TRAP LOOP",Constants.Intake.PID.trapLoop);
    }

    public void update() {
        switch (intakeState) {
            case IDLE:
                if(gamepad.getR1ButtonPressed()) {
                    intakeState = State.FORWARDS;
                }
                left.setControl(new DutyCycleOut(Constants.Intake.Storage.ROT_IDLE));
                right.setControl(new DutyCycleOut(Constants.Intake.Storage.ROT_IDLE));
                drive.setControl(new DutyCycleOut(Constants.Intake.Pivot.ROT_IDLE));
                break;
            case FORWARDS:
                if(gamepad.getR1ButtonPressed()) {
                    intakeState = State.REVERSE;
                }
                if(gamepad.getL1ButtonPressed()) {
                    intakeState = State.IDLE;
                }
                left.setControl(new DutyCycleOut(Constants.Intake.Storage.ROT_SPEED));
                right.setControl(new DutyCycleOut(Constants.Intake.Storage.ROT_SPEED));
                drive.setControl(new DutyCycleOut(Constants.Intake.Pivot.ROT_SPEED));
                break;
            case REVERSE:
                if(gamepad.getL1ButtonPressed()) {
                    intakeState = State.IDLE;
                }
                if(gamepad.getR1ButtonPressed()) {
                    intakeState = State.FORWARDS;
                }
                left.setControl(new DutyCycleOut(-Constants.Intake.Storage.ROT_SPEED));
                right.setControl(new DutyCycleOut(-Constants.Intake.Storage.ROT_SPEED));
                drive.setControl(new DutyCycleOut(-Constants.Intake.Pivot.ROT_SPEED));
                break;
         
        }
        System.out.println(intakeState);

        request.withFeedForward(Constants.Intake.PID.kf);
        
        lift.setControl(request.withPosition(Constants.Intake.PID.TARGET_ROT).withSlot(0));

        updateDash();
        while(Constants.Intake.PID.trapLoop) {
            updateDash();
        }

        
    }
    public double getDriveIntakePos() {
        return drive.getPosition().getValueAsDouble();
    }
}

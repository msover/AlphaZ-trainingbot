package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.PS5Controller;
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
    private TalonFX pivot = Hardware.Intake.Pivot.pivot;

    private MotionMagicVoltage request;
    private long startTime;


    public void setPidConfig(TalonFX motor, double ks, double kv, double ka, double kp, double ki, double kd,
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
        
        // motorConfig.CurrentLimits.withSupplyCurrentLowerLimit(Amps.of(20));
        // motorConfig.CurrentLimits.withSupplyCurrentLimit(Amps.of(30));
        // motorConfig.CurrentLimits.withSupplyCurrentLowerTime(Seconds.of(0.2));
        // motorConfig.CurrentLimits.withSupplyCurrentLimitEnable(true);
        // motorConfig.CurrentLimits.withStatorCurrentLimit(Amps.of(45));
        // motorConfig.CurrentLimits.withStatorCurrentLimitEnable(true);

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
            pivot,
            Constants.Intake.PivotPID.ks,
            Constants.Intake.PivotPID.kv,
            Constants.Intake.PivotPID.ka,
            Constants.Intake.PivotPID.kp,
            Constants.Intake.PivotPID.ki,
            Constants.Intake.PivotPID.kd,
            Constants.Intake.PivotPID.cruiseVel,
            Constants.Intake.PivotPID.acc,
            Constants.Intake.PivotPID.jerk,
            true
        );
        setPidConfig(
            drive,
            Constants.Intake.DrivePID.ks,
            Constants.Intake.DrivePID.kv,
            Constants.Intake.DrivePID.ka,
            Constants.Intake.DrivePID.kp,
            Constants.Intake.DrivePID.ki,
            Constants.Intake.DrivePID.kd,
            Constants.Intake.DrivePID.cruiseVel,
            Constants.Intake.DrivePID.acc,
            Constants.Intake.DrivePID.jerk,
            true
        );
        setPidConfig(
            left,
            Constants.Intake.StoragePID.Left.ks,
            Constants.Intake.StoragePID.Left.kv,
            Constants.Intake.StoragePID.Left.ka,
            Constants.Intake.StoragePID.Left.kp,
            Constants.Intake.StoragePID.Left.ki,
            Constants.Intake.StoragePID.Left.kd,
            Constants.Intake.StoragePID.Left.cruiseVel,
            Constants.Intake.StoragePID.Left.acc,
            Constants.Intake.StoragePID.Left.jerk,
            true
        );
        setPidConfig(
            right,
            Constants.Intake.StoragePID.Right.ks,
            Constants.Intake.StoragePID.Right.kv,
            Constants.Intake.StoragePID.Right.ka,
            Constants.Intake.StoragePID.Right.kp,
            Constants.Intake.StoragePID.Right.ki,
            Constants.Intake.StoragePID.Right.kd,
            Constants.Intake.StoragePID.Right.cruiseVel,
            Constants.Intake.StoragePID.Right.acc,
            Constants.Intake.StoragePID.Right.jerk,
            false
        );
        pivot.setPosition(11);
        startTime = System.currentTimeMillis();
        initDash();
    }


    private void initDash() {
        SmartDashboard.putNumber("target rot", Constants.Intake.PivotPID.TARGET_ROT);
        SmartDashboard.putNumber("kf", Constants.Intake.PivotPID.kf);
        SmartDashboard.putNumber("ks", Constants.Intake.PivotPID.ks);
        SmartDashboard.putNumber("kv", Constants.Intake.PivotPID.kv);
        SmartDashboard.putNumber("ka", Constants.Intake.PivotPID.ka);
        SmartDashboard.putNumber("kp", Constants.Intake.PivotPID.kp);
        SmartDashboard.putNumber("ki", Constants.Intake.PivotPID.ki);
        SmartDashboard.putNumber("kd", Constants.Intake.PivotPID.kd);
        SmartDashboard.putNumber("cruiseVel", Constants.Intake.PivotPID.cruiseVel);
        SmartDashboard.putNumber("accell", Constants.Intake.PivotPID.acc);
        SmartDashboard.putNumber("jerk", Constants.Intake.PivotPID.jerk);
        SmartDashboard.putBoolean("TRAP LOOP", Constants.Intake.PivotPID.trapLoop);
    }
    private void updateDash() {
        SmartDashboard.putNumber("pos", pivot.getPosition().getValueAsDouble());
        Constants.Intake.PivotPID.TARGET_ROT = SmartDashboard.getNumber("target rot", Constants.Intake.PivotPID.TARGET_ROT);
        Constants.Intake.PivotPID.kf = SmartDashboard.getNumber("kf", Constants.Intake.PivotPID.kf);
        Constants.Intake.PivotPID.ks = SmartDashboard.getNumber("ks", Constants.Intake.PivotPID.ks);
        Constants.Intake.PivotPID.kv = SmartDashboard.getNumber("kv", Constants.Intake.PivotPID.kv);
        Constants.Intake.PivotPID.ka = SmartDashboard.getNumber("ka", Constants.Intake.PivotPID.ka);
        Constants.Intake.PivotPID.kp = SmartDashboard.getNumber("kp", Constants.Intake.PivotPID.kp);
        Constants.Intake.PivotPID.ki = SmartDashboard.getNumber("ki", Constants.Intake.PivotPID.ki);
        Constants.Intake.PivotPID.kd = SmartDashboard.getNumber("kd", Constants.Intake.PivotPID.kd);
        Constants.Intake.PivotPID.cruiseVel = (int) SmartDashboard.getNumber("cruiseVel", Constants.Intake.PivotPID.cruiseVel);
        Constants.Intake.PivotPID.acc = (int) SmartDashboard.getNumber("accell", Constants.Intake.PivotPID.acc);
        Constants.Intake.PivotPID.jerk = (int) SmartDashboard.getNumber("jerk", Constants.Intake.PivotPID.jerk);
        Constants.Intake.PivotPID.trapLoop = SmartDashboard.getBoolean("TRAP LOOP",Constants.Intake.PivotPID.trapLoop);
    }

    public void update(PS5Controller gamepad) {
        this.gamepad = gamepad;
        if (System.currentTimeMillis() - startTime > Constants.Outtake.Lift.TIMEOUT_MS) {
            switch (intakeState) {
                case IDLE:
                    if(gamepad.getR1ButtonPressed()) {
                        intakeState = State.FORWARDS;
                    }
                    Constants.Intake.PivotPID.TARGET_ROT = 11;
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
                    Constants.Intake.PivotPID.TARGET_ROT = -0.5;
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
                    Constants.Intake.PivotPID.TARGET_ROT = -0.5;
                    left.setControl(new DutyCycleOut(-Constants.Intake.Storage.ROT_SPEED));
                    right.setControl(new DutyCycleOut(-Constants.Intake.Storage.ROT_SPEED));
                    drive.setControl(new DutyCycleOut(-Constants.Intake.Pivot.ROT_SPEED));
                    break;
            }
        }

        request.withFeedForward(Constants.Intake.PivotPID.kf*Math.cos(Math.toRadians((90/8.35)*pivot.getPosition().getValueAsDouble())));
        
        pivot.setControl(request.withPosition(Constants.Intake.PivotPID.TARGET_ROT).withSlot(0));


        
        updateDash();
        while(Constants.Intake.PivotPID.trapLoop) {
            updateDash();
        }

        
    }
    public double getDriveIntakePos() {
        return drive.getPosition().getValueAsDouble();
    }
}

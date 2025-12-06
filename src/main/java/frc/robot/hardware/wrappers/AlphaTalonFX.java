package frc.robot.hardware.wrappers;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class AlphaTalonFX {
    public TalonFX motor;
    private final VoltageOut voltageOut = new VoltageOut(0);
    final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    boolean reversed = false;
    TalonFXConfiguration motorConfig;
    double ks = 0;
    double kv = 0;
    double ka = 0;
    double kp = 0;
    double ki = 0;
    double kd = 0;

    double vel = 0;
    double acc = 0;
    double jerk = 0;

    public AlphaTalonFX(int deviceId, String bus, boolean reversed) {
        motor = new TalonFX(deviceId, bus);
        this.reversed = reversed;

        motorConfig = new TalonFXConfiguration();

        applyMotorConfig();
    }

    public AlphaTalonFX(int deviceId, String bus, boolean reversed, double supplyCurrent, double statorCurrent) {
        motor = new TalonFX(deviceId, bus);
        this.reversed = reversed;

        motorConfig = new TalonFXConfiguration();

        motorConfig.CurrentLimits.withSupplyCurrentLimitEnable(true);
        motorConfig.CurrentLimits.withStatorCurrentLimitEnable(true);
        motorConfig.CurrentLimits.withSupplyCurrentLimit(Amps.of(supplyCurrent));
        motorConfig.CurrentLimits.withStatorCurrentLimit(Amps.of(statorCurrent));

        applyMotorConfig();
    }

    public AlphaTalonFX(int deviceId, String bus, boolean reversed, double ks, double kv, double ka, double kp, double ki, double kd, double vel, double acc, double jerk) {
        motor = new TalonFX(deviceId, bus);
        this.reversed = reversed;

        motorConfig = new TalonFXConfiguration();

        setMotionMagicCoefficients(ks, kv, ka, kp, ki, kd, vel, acc, jerk);

        updateMotionMagicCoefficients();

        applyMotorConfig();
    }

    public AlphaTalonFX(int deviceId, String bus, boolean reversed, double ks, double kv, double ka, double kp, double ki, double kd, double vel, double acc, double jerk, double supplyCurrent, double statorCurrent) {
        motor = new TalonFX(deviceId, bus);
        this.reversed = reversed;

        motorConfig = new TalonFXConfiguration();

        setMotionMagicCoefficients(ks, kv, ka, kp, ki, kd, vel, acc, jerk);

        updateMotionMagicCoefficients();

        motorConfig.CurrentLimits.withSupplyCurrentLimitEnable(true);
        motorConfig.CurrentLimits.withStatorCurrentLimitEnable(true);
        motorConfig.CurrentLimits.withSupplyCurrentLimit(Amps.of(supplyCurrent));
        motorConfig.CurrentLimits.withStatorCurrentLimit(Amps.of(statorCurrent));

        applyMotorConfig();
    }

    public void setEncoderPosition(double position) {
        motor.setPosition(0);
    }

    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    public double getPositionError() {
        return motor.getClosedLoopError().getValueAsDouble();
    }

    public void setVoltage(double voltage) {
        motor.setControl(voltageOut.withOutput(voltage));
    }

    public void setNormalizedVoltage(double normalizedVoltage) {
        motor.setControl(voltageOut.withOutput(normalizedVoltage*12));
    }

    public void setMotionMagicPosition(double position) {
        motor.setControl(motionMagicVoltage.withPosition(position));
    }

    public void setFeedForward(double ff) {
        motionMagicVoltage.withFeedForward(ff);
    }

    public void setMotionMagicCoefficients(double ks, double kv, double ka, double kp, double ki, double kd, double vel, double acc, double jerk) {
        this.ks = ks;
        this.kv = kv;
        this.ka = ka;
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.vel = vel;
        this.acc = acc;
        this.jerk = jerk;
    }

    public void updateMotionMagicCoefficients() {
        motorConfig.Slot0.kS = ks;
        motorConfig.Slot0.kV = kv;
        motorConfig.Slot0.kA = ka;
        motorConfig.Slot0.kP = kp;
        motorConfig.Slot0.kI = ki;
        motorConfig.Slot0.kD = kd;

        motorConfig.MotionMagic.MotionMagicCruiseVelocity = vel;
        motorConfig.MotionMagic.MotionMagicAcceleration = acc;
        motorConfig.MotionMagic.MotionMagicJerk = jerk;
    }

    public void applyMotorConfig() {

        motorConfig.Voltage.PeakForwardVoltage = 12;
        motorConfig.Voltage.PeakReverseVoltage = -12;

        if (reversed) {
            motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        } else {
            motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }

        motor.getConfigurator().apply(motorConfig);
    }

    public double getStatorCurrent() {
        return motor.getStatorCurrent().getValueAsDouble();
    }

    public double getSupplyCurrent() {
        return motor.getSupplyCurrent().getValueAsDouble();
    }

    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    public double getVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    public double getSupplyVoltage() {
        return motor.getSupplyVoltage().getValueAsDouble();
    }

}

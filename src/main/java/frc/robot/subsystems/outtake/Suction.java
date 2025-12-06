package frc.robot.subsystems.outtake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.hardware.RobotHardware;
import frc.robot.subsystems.Subsystem;

public class Suction implements Subsystem {

    private final RobotHardware robot = RobotHardware.getInstance();

    public enum SuctionState {
        RELEASE,
        SUCK
    }

    public SuctionState suctionState = SuctionState.RELEASE;

    public static double RELEASE = 0;
    public static double SUCK = 4.8;

    public static double suckTimeout = 0.2;
    public static double errorThreshold = 0.1;

    public static double ks = 0;
    public static double kv = 0;
    public static double ka = 0;
    public static double kp = 10;
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 0;

    public static double vel = 40;
    public static double acc = 100;
    public static double jerk = 1000;

    public static double target = 0;

    @Override
    public void initialize() {
        robot.suctionMotor.setEncoderPosition(0);
        initTelemetry();
    }

    @Override
    public void loop() {

        //TODO comment this after done tuning
        // robot.suctionMotor.setMotionMagicCoefficients(ks, kv, ka, kp, ki, kd, vel, acc, jerk);

        // robot.suctionMotor.updateMotionMagicCoefficients();

        // robot.suctionMotor.applyMotorConfig();

        updateState();
        updateTelemetry();
    }
    
    public void setState(SuctionState suctionState) {
        this.suctionState = suctionState;
        switch (suctionState) {
            case RELEASE:
                target = RELEASE;
                robot.suctionMotor.setMotionMagicPosition(RELEASE);
                break;
            case SUCK:
                target = SUCK;
                robot.suctionMotor.setMotionMagicPosition(SUCK);
                break;
        }
    }

    public void updateState() {
        switch (suctionState) {
            case RELEASE:
                break;
            case SUCK:
                break;
        }
    }

    public SuctionState getState() {
        return suctionState;
    }

    public double getPositionError() {
        return robot.suctionMotor.getPositionError();
    }

    public boolean inPosition(double error) {
        return getPositionError() < error;
    }

    public void initTelemetry() {
        SmartDashboard.putNumber("suction ks", ks);
        SmartDashboard.putNumber("suction kv", kv);
        SmartDashboard.putNumber("suction ka", ka);
        SmartDashboard.putNumber("suction kp", kp);
        SmartDashboard.putNumber("suction ki", ki);
        SmartDashboard.putNumber("suction kd", kd);
        SmartDashboard.putNumber("suction kf", kf);
        SmartDashboard.putNumber("suction vel", vel);
        SmartDashboard.putNumber("suction acc", acc);
        SmartDashboard.putNumber("suction jerk", jerk);
    }

    public void updateTelemetry() {
        SmartDashboard.putNumber("suction position", robot.suctionMotor.getPosition());
        SmartDashboard.putNumber("suction target", target);
        SmartDashboard.putNumber("suction error",getPositionError());
        SmartDashboard.putNumber("suction supply current", robot.suctionMotor.getSupplyCurrent());
        SmartDashboard.putNumber("suction stator current", robot.suctionMotor.getStatorCurrent());
        SmartDashboard.putNumber("suction voltage", robot.suctionMotor.getVoltage());
        SmartDashboard.putNumber("suction supply voltage", robot.suctionMotor.getSupplyVoltage());
        ks = SmartDashboard.getNumber("suction ks", ks);
        kv = SmartDashboard.getNumber("suction kv", kv);
        ka = SmartDashboard.getNumber("suction ka", ka);
        kp = SmartDashboard.getNumber("suction kp", kp);
        ki = SmartDashboard.getNumber("suction ki", ki);
        kd = SmartDashboard.getNumber("suction kd", kd);
        kf = SmartDashboard.getNumber("suction kf", kf);
        vel = SmartDashboard.getNumber("suction vel", vel);
        acc = SmartDashboard.getNumber("suction acc", acc);
        jerk = SmartDashboard.getNumber("suction jerk", jerk);
    }
}

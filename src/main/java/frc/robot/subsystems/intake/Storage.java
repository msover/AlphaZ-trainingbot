package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.hardware.RobotHardware;
import frc.robot.subsystems.Subsystem;

public class Storage implements Subsystem {
    private final RobotHardware robot = RobotHardware.getInstance();

    public enum StorageState {
        IDLE,
        INTAKE,
        REVERSE
    }

    public StorageState storageState = StorageState.IDLE;

    public static double IDLE = 0;
    public static double INTAKE = 0.5;
    public static double REVERSE = -0.5;

    @Override
    public void initialize() {
        //initTelemetry();
    }

    @Override
    public void loop() {
        updateState();
        //updateTelemetry();
    }

    public void setState(StorageState storageState) {
        this.storageState = storageState;
        switch (storageState) {
            case IDLE:
                robot.leftStorageMotor.setNormalizedVoltage(IDLE);
                robot.rightStorageMotor.setNormalizedVoltage(IDLE);
                break;
            case INTAKE:
                robot.leftStorageMotor.setNormalizedVoltage(INTAKE);
                robot.rightStorageMotor.setNormalizedVoltage(INTAKE);
                break;
            case REVERSE:
                robot.leftStorageMotor.setNormalizedVoltage(REVERSE);
                robot.rightStorageMotor.setNormalizedVoltage(REVERSE);
                break;
        }
    }

    public void updateState() {
        switch (storageState) {
            case IDLE:
                break;
            case INTAKE:
                break;
            case REVERSE:
                break;
        }
    }

    public void initTelemetry() {

    }

    public void updateTelemetry() {
        SmartDashboard.putNumber("left storage velocity", robot.leftStorageMotor.getVelocity());
        SmartDashboard.putNumber("right storage velocity", robot.rightStorageMotor.getVelocity());
        SmartDashboard.putNumber("left storage supply current", robot.leftStorageMotor.getSupplyCurrent());
        SmartDashboard.putNumber("right storage supply current", robot.rightStorageMotor.getSupplyCurrent());
        SmartDashboard.putNumber("left storage stator current", robot.leftStorageMotor.getStatorCurrent());
        SmartDashboard.putNumber("right storage stator current", robot.rightStorageMotor.getStatorCurrent());
        SmartDashboard.putNumber("left storage voltage", robot.leftStorageMotor.getVoltage());
        SmartDashboard.putNumber("right storage voltage", robot.rightStorageMotor.getVoltage());
        SmartDashboard.putNumber("left storage supply voltage", robot.leftStorageMotor.getSupplyVoltage());
        SmartDashboard.putNumber("right storage supply voltage", robot.rightStorageMotor.getSupplyVoltage());
    }
}

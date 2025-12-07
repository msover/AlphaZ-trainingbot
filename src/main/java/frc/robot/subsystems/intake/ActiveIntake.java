package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.hardware.RobotHardware;
import frc.robot.subsystems.Subsystem;

public class ActiveIntake implements Subsystem {

    private final RobotHardware robot = RobotHardware.getInstance();

    public enum ActiveIntakeState {
        IDLE,
        INTAKE,
        REVERSE
    }

    public ActiveIntakeState activeIntakeState = ActiveIntakeState.IDLE;

    public static double IDLE = 0;
    public static double INTAKE = 0.6;
    public static double REVERSE = -0.6;

    @Override
    public void initialize() {
        //initTelemetry();
    }

    @Override
    public void loop() {
        updateState();
        //updateTelemetry();
    }

    public void setState(ActiveIntakeState activeIntakeState) {
        this.activeIntakeState = activeIntakeState;
        switch (activeIntakeState) {
            case IDLE:
                robot.activeIntakeMotor.setNormalizedVoltage(IDLE);
                break;
            case INTAKE:
                robot.activeIntakeMotor.setNormalizedVoltage(INTAKE);
                break;
            case REVERSE:
                robot.activeIntakeMotor.setNormalizedVoltage(REVERSE);
                break;
        }
    }

    public void updateState() {
        switch (activeIntakeState) {
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
        SmartDashboard.putNumber("active intake velocity", robot.activeIntakeMotor.getVelocity());
        SmartDashboard.putNumber("active intake supply current", robot.activeIntakeMotor.getSupplyCurrent());
        SmartDashboard.putNumber("active intake stator current", robot.activeIntakeMotor.getStatorCurrent());
        SmartDashboard.putNumber("active intake voltage", robot.activeIntakeMotor.getVoltage());
        SmartDashboard.putNumber("active intake supply voltage", robot.activeIntakeMotor.getSupplyVoltage());
    }

}

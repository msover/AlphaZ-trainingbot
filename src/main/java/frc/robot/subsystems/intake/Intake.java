package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.PS5Controller;
import frc.robot.utils.hardware.Hardware;

public class Intake {

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

    public void init(PS5Controller gamepad) {
        this.gamepad = gamepad;

        drive.setControl(new DutyCycleOut(0));
        ///lift.setControl(new DutyCycleOut(0));
        left.setControl(new DutyCycleOut(0));
        right.setControl(new DutyCycleOut(0));
    }
    public void update(double pivotPow, double storagePow, double pivotPos) {
        drive.setControl(new DutyCycleOut(-pivotPow));
        left.setControl(new DutyCycleOut(-storagePow));
        right.setControl(new DutyCycleOut(storagePow));
        
    }
    public double getDriveIntakePos() {
        return drive.getPosition().getValueAsDouble();
    }
}

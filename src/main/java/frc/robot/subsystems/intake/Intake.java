package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.utils.hardware.Hardware;

public class Intake {
    
    private static Intake instance;
    public static Intake getInstance() {
        if(instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private TalonFX left = Hardware.Intake.Storage.left;
    private TalonFX right = Hardware.Intake.Storage.right;
    private TalonFX drive = Hardware.Intake.Pivot.drive;

    public void init() {
        drive.setControl(new DutyCycleOut(0));
        ///lift.setControl(new DutyCycleOut(0));
        left.setControl(new DutyCycleOut(0));
        right.setControl(new DutyCycleOut(0));
    }
    public void update(double pivotPow, double pivotPos, double storagePow) {
        drive.setControl(new DutyCycleOut(pivotPow));
        left.setControl(new DutyCycleOut(storagePow));
        right.setControl(new DutyCycleOut(storagePow));
        
    }
    public double getDriveIntakePos() {
        return drive.getPosition().getValueAsDouble();
    }
}

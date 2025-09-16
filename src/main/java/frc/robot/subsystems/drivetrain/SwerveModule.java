package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.utils.PID.DrivetrainPID;
import frc.robot.utils.PID.DrivetrainPID.AnglePID;

public class SwerveModule {
    private MotorController driveMotor;
    private MotorController angleMotor;
    private CANcoder angleEncoder;
    private AnglePID anglePID = new DrivetrainPID().new AnglePID(angleMotor, angleEncoder);
    public SwerveModule(MotorController driveMotor, MotorController angleMotor, CANcoder angleEncoder) {
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.angleEncoder = angleEncoder;
    }
    
    public void update(double driveSpeed, Rotation2d desiredAngle) {
        driveMotor.set(driveSpeed);
        anglePID.update(desiredAngle.getRadians());
    }
}

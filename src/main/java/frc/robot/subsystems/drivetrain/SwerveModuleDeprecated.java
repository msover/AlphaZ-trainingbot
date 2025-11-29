package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.utils.PID.DrivetrainPIDDeprecated;
import frc.robot.utils.PID.DrivetrainPIDDeprecated.AnglePID;
@Deprecated
public class SwerveModuleDeprecated {
    private MotorController driveMotor;
    private MotorController angleMotor;
    private CANcoder angleEncoder;
    private AnglePID anglePID = new DrivetrainPIDDeprecated().new AnglePID(angleMotor, angleEncoder);
    public SwerveModuleDeprecated(MotorController driveMotor, MotorController angleMotor, CANcoder angleEncoder) {
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.angleEncoder = angleEncoder;
    }
    
    public void update(double driveSpeed, Rotation2d desiredAngle) {
        driveMotor.set(driveSpeed);
        anglePID.update(desiredAngle.getRadians());
    }
}

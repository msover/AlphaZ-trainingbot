package frc.robot.utils.PID;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.utils.constants.Constants;

public class DrivetrainPID {
    public class AnglePID {
        private PIDController controller;
        
        MotorController motor;
        CANcoder encoder;
        public AnglePID(MotorController motor, CANcoder encoder) {
            this.motor = motor;
            this.encoder = encoder;
            controller = new PIDController(Constants.Drivetrain.SwerveModule.angleP, Constants.Drivetrain.SwerveModule.angleI, Constants.Drivetrain.SwerveModule.angleD);
            controller.enableContinuousInput(-180, 180);
        }
        
        public void update(double targetAngle) {
            double pow = controller.calculate(encoder.getAbsolutePosition().getValueAsDouble(), targetAngle);
            pow = Math.max(Math.min(pow, 1.0), -1.0);
            motor.set(pow);
        }
    }
}

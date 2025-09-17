package frc.robot.utils.hardware;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.DeviceConstructor;

import edu.wpi.first.wpilibj.Encoder;
import frc.robot.utils.constants.Constants;

public class Hardware {
    public class Drivetrain {
        
        public class LeftBack {
            TalonFX driveMotor = new TalonFX(Constants.Drivetrain.LeftBack.driveMotorID);
            TalonFX steerMotor = new TalonFX(Constants.Drivetrain.LeftBack.steerMotorID);
            CANcoder steerEncoder = new CANcoder(Constants.Drivetrain.LeftBack.steerEncoderID);

            DeviceConstructor<TalonFX> driveMotorConstructor = (id, name) -> driveMotor;
            DeviceConstructor<TalonFX> steerMotorConstructor = (id, name) -> steerMotor;
            DeviceConstructor<CANcoder> steerEncoderConstructor = (id, name) -> steerEncoder;

            SwerveDrivetrain<TalonFX, TalonFX, CANcoder> module = new SwerveDrivetrain<>(driveMotorConstructor, steerMotorConstructor,
            steerEncoderConstructor, null, 0.0);
        }
        public class LeftFront {
            TalonFX driveMotor = new TalonFX(Constants.Drivetrain.LeftFront.driveMotorID);
            TalonFX steerMotor = new TalonFX(Constants.Drivetrain.LeftFront.steerMotorID);
            CANcoder steerEncoder = new CANcoder(Constants.Drivetrain.LeftFront.steerEncoderID);

            DeviceConstructor<TalonFX> driveMotorConstructor = (id, name) -> driveMotor;
            DeviceConstructor<TalonFX> steerMotorConstructor = (id, name) -> steerMotor;
            DeviceConstructor<CANcoder> steerEncoderConstructor = (id, name) -> steerEncoder;

            SwerveDrivetrain<TalonFX, TalonFX, CANcoder> module = new SwerveDrivetrain<>(driveMotorConstructor, steerMotorConstructor,
            steerEncoderConstructor, null, 0.0);
        }
        public class RightBack {
            TalonFX driveMotor = new TalonFX(Constants.Drivetrain.RightBack.driveMotorID);
            TalonFX steerMotor = new TalonFX(Constants.Drivetrain.RightBack.steerMotorID);
            CANcoder steerEncoder = new CANcoder(Constants.Drivetrain.RightBack.steerEncoderID);

            DeviceConstructor<TalonFX> driveMotorConstructor = (id, name) -> driveMotor;
            DeviceConstructor<TalonFX> steerMotorConstructor = (id, name) -> steerMotor;
            DeviceConstructor<CANcoder> steerEncoderConstructor = (id, name) -> steerEncoder;

            SwerveDrivetrain<TalonFX, TalonFX, CANcoder> module = new SwerveDrivetrain<>(driveMotorConstructor, steerMotorConstructor,
            steerEncoderConstructor, null, 0.0);
        }
        public class RightFront {
            TalonFX driveMotor = new TalonFX(Constants.Drivetrain.RightFront.driveMotorID);
            TalonFX steerMotor = new TalonFX(Constants.Drivetrain.RightFront.steerMotorID);
            CANcoder steerEncoder = new CANcoder(Constants.Drivetrain.RightFront.steerEncoderID);

            DeviceConstructor<TalonFX> driveMotorConstructor = (id, name) -> driveMotor;
            DeviceConstructor<TalonFX> steerMotorConstructor = (id, name) -> steerMotor;
            DeviceConstructor<CANcoder> steerEncoderConstructor = (id, name) -> steerEncoder;

            SwerveDrivetrain<TalonFX, TalonFX, CANcoder> module = new SwerveDrivetrain<>(driveMotorConstructor, steerMotorConstructor,
            steerEncoderConstructor, null, 0.0);
        }

    }
    public class Intake{
        
    }
    public class Outtake {
        public class Lift {
            public static TalonFX liftMotor = new TalonFX(Constants.Outtake.Lift.liftMotorID);
            public static Encoder liftEncoder = new Encoder(Constants.Outtake.Lift.liftEncoderAID, Constants.Outtake.Lift.liftEncoderBID);
            public static double targetPos;
        }
    }
    
}

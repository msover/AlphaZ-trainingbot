package frc.robot.subsystems.hardware;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

public class Hardware {
    public class Drivetrain {
        public class LeftBack {
            public static PWMVictorSPX driveMotor = new PWMVictorSPX(0);
            public static PWMVictorSPX angleMotor = new PWMVictorSPX(0);
            public static AnalogEncoder angleEncoder = new AnalogEncoder(0);
        }
        public class LeftFront {
            public static PWMVictorSPX driveMotor = new PWMVictorSPX(0);
            public static PWMVictorSPX angleMotor = new PWMVictorSPX(0);
            public static AnalogEncoder angleEncoder = new AnalogEncoder(0);
        }
        public class RightBack {
            public static PWMVictorSPX driveMotor = new PWMVictorSPX(0);
            public static PWMVictorSPX angleMotor = new PWMVictorSPX(0);
            public static AnalogEncoder angleEncoder = new AnalogEncoder(0);
        }
        public class RightFront {
            public static PWMVictorSPX driveMotor = new PWMVictorSPX(0);
            public static PWMVictorSPX angleMotor = new PWMVictorSPX(0);
            public static AnalogEncoder angleEncoder = new AnalogEncoder(0);
        }
    }
    public class Intake{
        
    }
    public class Outtake {
        public class Lift {
            public static PWMVictorSPX liftMotor = new PWMVictorSPX(0);
            public static Encoder liftEncoder = new Encoder(0, 0);
            public static double targetPos;
        }
    }
    
}

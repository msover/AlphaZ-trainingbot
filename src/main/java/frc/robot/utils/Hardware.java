package frc.robot.utils;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

public class Hardware {
    public class Outtake {
        public class Lift {
            public static PWMVictorSPX liftMotor = new PWMVictorSPX(0);
            public static Encoder liftEncoder = new Encoder(0, 1);
            public static double targetPos;
        }
    }
}

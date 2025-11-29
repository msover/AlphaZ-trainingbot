package frc.robot.utils.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Constants {
    public static class Drivetrain {
        public static class LeftBack {
            public static int driveMotorID = 0;
            public static int steerMotorID = 0;
            public static int steerEncoderID = 0;
        }
        public static class LeftFront {
            public static int driveMotorID = 0;
            public static int steerMotorID = 0;
            public static int steerEncoderID = 0;
        }
        public static class RightBack {
            public static int driveMotorID = 0;
            public static int steerMotorID = 0;
            public static int steerEncoderID = 0;
        }
        public static class RightFront {
            public static int driveMotorID = 0;
            public static int steerMotorID = 0;
            public static int steerEncoderID = 0;
        }
        
    }
    public static class Intake {
        public static class Pivot {
            public static int driveId = 51;
            public static int liftId = 52;
        }
        public static class Storage {
            public static int leftId = 61;
            public static int rightId = 62;
        }
    }

    public static class Outtake {
        public static class Lift {
            public static int leftMotorId = 17;
            public static int rightMotorId = 27;

            public static double LEFT_IN = 0;
            public static double RIGHT_IN = 0;

            public static double LEFT_TRANSFER = 1.1;
            public static double RIGHT_TRANSFER = 1.1;

            public static double LEFT_SAFE = 3;
            public static double RIGHT_SAFE = 3;

            public static double LEFT_SCORE1 = 2.3;
            public static double RIGHT_SCORE1 = 6;

            public static double LEFT_SCORE2 = 9;
            public static double RIGHT_SCORE2 = 12;

            public static double LEFT_SCORE3 = 12;
            public static double RIGHT_SCORE3 = 16;
            
            public static double TARGET_ROT = 0;
            
            public static double ks = 0;
            public static double kv = 0;
            public static double ka = 0;
            public static double kp = 0;
            public static double ki = 0;
            public static double kd = 0;
            public static double kf = 0;

            
            public static int cruiseVel = 0;
            public static int acc = 0;
            public static int jerk = 0;

            
        }
    }
}

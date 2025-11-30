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
            public static int pivotId = 52;

            public static double ROT_SPEED = 0.6;
            public static double ROT_IDLE = 0;
        }
        public static class Storage {
            public static int leftId = 61;
            public static int rightId = 62;
            public static double ROT_SPEED = 0.5;
            public static double ROT_IDLE = 0;
        }
        public static class PivotPID {
            public static double TARGET_ROT = 11;
            public static double ks = 0;
            public static double kv = 0.12;
            public static double ka = 0.01;
            public static double kp = 4;
            public static double ki = 0;
            public static double kd = 0.1;
            public static double kf = 0.31;

            
            public static int cruiseVel = 30;
            public static int acc = 45;
            public static int jerk = 450;

            public static boolean trapLoop = false;
        }
        public static class DrivePID {
            public static double TARGET_ROT = 0;
            public static double ks = 0;
            public static double kv = 0;
            public static double ka = 0;
            public static double kp = 0;
            public static double ki = 0;
            public static double kd = 0;
            public static double kf = 0;

            
            public static int cruiseVel = 125;
            public static int acc = 25;
            public static int jerk = 25;

            public static boolean trapLoop = false;
        }
        public static class StoragePID {
            public static class Left {
                public static double TARGET_ROT = 0;
                public static double ks = 0;
                public static double kv = 0;
                public static double ka = 0;
                public static double kp = 0;
                public static double ki = 0;
                public static double kd = 0;
                public static double kf = 0;
    
                
                public static int cruiseVel = 125;
                public static int acc = 25;
                public static int jerk = 25;
    
                public static boolean trapLoop = false;
            }
            public static class Right {
                public static double TARGET_ROT = 0;
                public static double ks = 0;
                public static double kv = 0;
                public static double ka = 0;
                public static double kp = 0;
                public static double ki = 0;
                public static double kd = 0;
                public static double kf = 0;
    
                
                public static int cruiseVel = 125;
                public static int acc = 25;
                public static int jerk = 25;
    
                public static boolean trapLoop = false;
            }
        }
    }

    public static class Outtake {
        public static class LiftPID {
            public static double TARGET_ROT = 0;
            public static double ks = 0;
            public static double kv = 0.12;
            public static double ka = 0.01;
            public static double kp = 13.5;
            public static double ki = 0;
            public static double kd = 0;
            public static double kf = 0.25;

            
            public static int cruiseVel = 35;
            public static int acc = 150;
            public static int jerk = 1000;

            public static double PERMISSIVE_COLLECTION_ERROR = 0.4;
            public static double PERMISSIVE_ERROR = 0.1;
            public static boolean trapLoop = false;
        }
        public static class SugativPID {
            public static double TARGET_ROT = 0;
            public static double ks = 0;
            public static double kv = 0;
            public static double ka = 0;
            public static double kp = 10;
            public static double ki = 0;
            public static double kd = 0;
            public static double kf = 0;

            
            public static int cruiseVel = 40;
            public static int acc = 100;
            public static int jerk = 1000;
            
            public static double PERMISSIVE_ERROR = 0.1;

            public static boolean trapLoop = false;
        }
        
        public static class Lift {
            public static int leftMotorId = 17;
            public static int rightMotorId = 27;
            public static int sugativMotorId = 37;

            public static long TIMEOUT_MS = 150;
            public static double IN = 0;

            public static double LEFT_TRANSFER = 1.1;
            public static double RIGHT_TRANSFER = 1.1;

            public static double SAFE = 3;
            public static double TRANSFER = 0.5;
            public static double SCORE1 = 4.5;
            public static double SCORE1_DISPARITY = 1.5;

            public static double SCORE2 = 10.5;
            public static double SCORE2_DISPARITY = 1.5; //12

            public static double SCORE3 = 14;
            public static double SCORE3_DISPARITY = 2;
            
            public static double TARGET_ROT_RIGHT = 0;
            public static double TARGET_ROT_LEFT = 0;

            public static double SUGATIV_SUGE = 4.8;
            public static double SUGATIV_LASA = 0;
        }
    }
}

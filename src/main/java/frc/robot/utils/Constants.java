package frc.robot.utils;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Constants {
    public static class Outtake {
        public static class Lift {
            public static int liftUp = 1;
            public static int liftDown = 0;

            public static final double p = 0.0;
            public static final double i = 0.0;
            public static final double d = 0.0;

            public static class Fmap {
                private static InterpolatingDoubleTreeMap fMap = new InterpolatingDoubleTreeMap();

                public static void setMap() {
                    fMap.put(null, null);
                    fMap.put(null, null);
                    fMap.put(null, null);
                    fMap.put(null, null);
                    fMap.put(null, null);
                }

                public static double getMap(double key) {
                    return fMap.get(key);
                }
            }
        }
    }
}

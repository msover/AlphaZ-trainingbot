package frc.robot.utils.PID.Outtake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import frc.robot.utils.Constants;

public class LiftPID {
    private static PIDController controller = new PIDController(Constants.Outtake.Lift.p, Constants.Outtake.Lift.i, Constants.Outtake.Lift.d);
    public static double getF(double pos) {
        return Constants.Outtake.Lift.Fmap.getMap(pos);
    }
    public static void init() {
        Constants.Outtake.Lift.Fmap.setMap();
    }
    public static void update(PWMVictorSPX motor, Encoder encoder, double targetPos) {
        double pow = controller.calculate(encoder.get(), targetPos) + getF(encoder.get());
        pow = Math.max(Math.min(pow, 1.0), -1.0);
        motor.set(pow);
    }
}

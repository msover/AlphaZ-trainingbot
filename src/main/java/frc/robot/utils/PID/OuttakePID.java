package frc.robot.utils.PID;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.utils.constants.Constants;

public class OuttakePID {
    public class LiftPID {
        private PIDController controller;

        MotorController motor;
        Encoder encoder;

        public LiftPID(MotorController motor, Encoder encoder) {
            this.motor = motor;
            this.encoder = encoder;
            Constants.Outtake.Lift.Fmap.setMap();
            controller = new PIDController(Constants.Outtake.Lift.p, Constants.Outtake.Lift.i,
                    Constants.Outtake.Lift.d);
        }

        public double getF(double pos) {
            return Constants.Outtake.Lift.Fmap.getMap(pos);
        }

        public void update(double targetPos) {
            double pow = controller.calculate(encoder.get(), targetPos) + getF(encoder.get());
            pow = Math.max(Math.min(pow, 1.0), -1.0);
            motor.set(pow);
        }
    }
}
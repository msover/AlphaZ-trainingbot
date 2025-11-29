package frc.robot.utils.wrappers;

import edu.wpi.first.wpilibj.XboxController;

public final class Gamepad {
    private static XboxController controller1;
    private static XboxController controller2;

    private Gamepad() {
        controller1 = new XboxController(0);
        //controller2 = new XboxController(1);
        controller2 = null;
    }
    
    public static XboxController get1() {
        return controller1;
    }
    public static XboxController get2() {
        return controller2;
    }
}

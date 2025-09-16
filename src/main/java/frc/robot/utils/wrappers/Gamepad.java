package frc.robot.utils.wrappers;

import edu.wpi.first.wpilibj.XboxController;

public final class Gamepad {
    private static Gamepad instance;
    private Gamepad() {
        controller1 = new XboxController(0);
        //controller2 = new XboxController(1);
        controller2 = null;
    }
    public static Gamepad getInstance() {
        if (instance == null) {
            instance = new Gamepad();
        }
        return instance;
    }
    private XboxController controller1;
    private XboxController controller2;
    
    public XboxController get1() {
        return controller1;
    }
    public XboxController get2() {
        return controller2;
    }
}

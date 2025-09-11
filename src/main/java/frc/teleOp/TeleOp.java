package frc.teleOp;

public class TeleOp {
    private static TeleOp instance;
    private TeleOp() {
    }
    public static TeleOp getInstance() {
        if (instance == null) {
            instance = new TeleOp();
        }
        return instance;
    }

    public void init() {
        System.out.println("TeleOp Init");
    }
    public void loop() {
        System.out.println("TeleOp Loop");
    }
}

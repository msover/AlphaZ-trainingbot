package frc.autonomous;

public class Autonomous {
    private static Autonomous instance;
    private Autonomous() {
    }
    public static Autonomous getInstance() {
        if (instance == null) {
            instance = new Autonomous();
        }
        return instance;
    }

    public void init() {
        System.out.println("Autonomous Init");
    }
    public void loop() {
        System.out.println("Autonomous Loop");
    }
}

package frc.teleOp;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;

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
    private PS5Controller controller = new PS5Controller(0);
    public void init() {
        Intake.getInstance().init(controller);
        Outtake.getInstance().init(controller);
    }
    public void loop() {
        Intake.getInstance().update(controller);
        Outtake.getInstance().update(controller);

    }
}
///Error at edu.wpi.first.wpilibj.RobotBase.runRobot(RobotBase.java:440): The startCompetition() method (or methods called by it) should have handled the exception above. ﻿

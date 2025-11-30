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
    public void init() {
        Intake.getInstance().init(new PS5Controller(0));
        Outtake.getInstance().init(new PS5Controller(0));
    }
    public void loop() {
        Intake.getInstance().update();
        Outtake.getInstance().update();
    }
}
///Error at edu.wpi.first.wpilibj.RobotBase.runRobot(RobotBase.java:440): The startCompetition() method (or methods called by it) should have handled the exception above. ﻿

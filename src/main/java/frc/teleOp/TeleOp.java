package frc.teleOp;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.utils.wrappers.Gamepad;

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
    private Gamepad gamepads;
    public void init() {
        Intake.getInstance().init();
        Outtake.getInstance().init();
    }
    public void loop() {
        ///Intake.getInstance().update(0.6, 0.8, 0);
        Outtake.getInstance().update();
        SmartDashboard.putNumber("pos", Intake.getInstance().getDriveIntakePos());
    }
}
///Error at edu.wpi.first.wpilibj.RobotBase.runRobot(RobotBase.java:440): The startCompetition() method (or methods called by it) should have handled the exception above. ﻿

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.hardware.RobotHardware;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.outtake.Suction;
import frc.robot.util.Vector;

public class Robot extends TimedRobot {

  private final RobotHardware robot = RobotHardware.getInstance();

  private final PS5Controller gamepad = new PS5Controller(0);

  Timer timer = new Timer();

  double timeThreshold = 1;

  double testValue = 0;

  @Override
  public void robotInit() {
    robot.initializeHardware();
    robot.initializeActuators();
    timer.reset();
  }

  @Override
  public void robotPeriodic() {
    if (timer.get() > timeThreshold) {
      robot.loop();
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {

    if (timer.get() > timeThreshold) {

      if (gamepad.getPSButtonPressed()) {
        testValue += 1;
      }

      System.out.println(testValue);

      robot.drive.setTargetVector(new Vector(-gamepad.getLeftY(), -gamepad.getLeftX(), -gamepad.getRightX()));

      if (gamepad.getR1ButtonPressed()) {
        switch (robot.intake.getState()) {
          case UP:
              robot.intake.setState(Intake.State.INTAKE);
              break;
          case INTAKE:
              robot.intake.setState(Intake.State.REVERSE);
              break;
          case REVERSE:
              robot.intake.setState(Intake.State.INTAKE);
              break;
        }
      }

      if (gamepad.getL1ButtonPressed() && robot.intake.getState() != Intake.State.UP) {
        robot.intake.setState(Intake.State.UP);
      }

      if (gamepad.getTouchpadButtonPressed()) {
        robot.outtake.setState(Outtake.State.TRANSFER);
      }

      if (gamepad.getCrossButtonPressed()) {
        if (robot.outtake.getState() == Outtake.State.PASSTHROUGH) {
          robot.outtake.setState(Outtake.State.SCORE_LOW);
        } else if (robot.outtake.getState() == Outtake.State.SCORE_LOW && robot.outtake.suction.getState() == Suction.SuctionState.SUCK) {
          robot.outtake.suction.setState(Suction.SuctionState.RELEASE);
        } else if (robot.outtake.getState() == Outtake.State.SCORE_LOW && robot.outtake.suction.getState() == Suction.SuctionState.RELEASE) {
          robot.outtake.setState(Outtake.State.PASSTHROUGH);
        }
      }
    }

  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}

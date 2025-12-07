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
import frc.robot.subsystems.outtake.Lift;
import frc.robot.subsystems.outtake.Suction;
import frc.robot.subsystems.outtake.Lift.LiftState;
import frc.robot.util.Vector;

public class Robot extends TimedRobot {

  private final RobotHardware robot;

  private final PS5Controller gamepad;

  Timer timer;

  double timeThreshold = 1;

  double testValue = 1;

  public Robot() {
    robot = RobotHardware.getInstance();
    gamepad = new PS5Controller(0);
    timer = new Timer();
  }

  @Override
  public void robotInit() {
    robot.initializeHardware();
    robot.initializeActuators();
    timer.reset();
    timer.start();
  }

  @Override
  public void robotPeriodic() {
    //DO NOT PUT ANYTHING HERE!!!!!!!!!!!!!
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
  public void teleopInit() {
    timer.reset();
    timer.start();
  }

  @Override
  public void teleopPeriodic() {

    if (timer.get() > timeThreshold) {

      robot.loop();

      robot.drive.setTargetVector(new Vector(-gamepad.getLeftY(), -gamepad.getLeftX(), -gamepad.getRightX()));

      if (gamepad.getTriangleButtonPressed()) {
        Lift.tuning = !Lift.tuning;
      }

      if (gamepad.getR1ButtonPressed()) {
        if (robot.intake.getState() == Intake.State.UP) {
          robot.intake.setState(Intake.State.INTAKE);
        } else if (robot.intake.getState() == Intake.State.INTAKE) {
          robot.intake.setState(Intake.State.REVERSE);
        } else if (robot.intake.getState() == Intake.State.REVERSE) {
          robot.intake.setState(Intake.State.INTAKE);
        }
      }

      if (gamepad.getL1ButtonPressed() && robot.intake.getState() != Intake.State.UP) {
        robot.intake.setState(Intake.State.UP);
      }

      if (gamepad.getTouchpadButtonPressed()) {
        robot.outtake.setState(Outtake.State.TRANSFER);
      }

      if (false) {
        if (robot.outtake.getState() == Outtake.State.PASSTHROUGH) {
          robot.outtake.setState(Outtake.State.SCORE_LOW);
        } else if (robot.outtake.getState() == Outtake.State.SCORE_LOW && robot.outtake.suction.getState() == Suction.SuctionState.SUCK) {
          robot.outtake.suction.setState(Suction.SuctionState.RELEASE);
        } else if (robot.outtake.getState() == Outtake.State.SCORE_LOW && robot.outtake.suction.getState() == Suction.SuctionState.RELEASE) {
          robot.outtake.setState(Outtake.State.PASSTHROUGH);
        }
      }

      if (gamepad.getSquareButtonPressed()) {
        //robot.outtake.lift.setState(LiftState.SCORE_LOW);
        robot.outtake.lift.targetLeft = 7;
        robot.outtake.lift.targetRight = 7;
      }

      if (gamepad.getCircleButtonPressed()) {
        robot.outtake.lift.targetLeft = 8.5;
        robot.outtake.lift.targetRight = 5.5;
      }

      if (gamepad.getCrossButtonPressed()) {
        robot.outtake.lift.targetLeft = 9;
        robot.outtake.lift.targetRight = 5;
      }

      if (gamepad.getPSButtonPressed()) {
        robot.outtake.lift.targetLeft = 3;
        robot.outtake.lift.targetRight = 3;
      }
    }

  }

  @Override
  public void teleopExit() {
    Lift.tuning = false;
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}

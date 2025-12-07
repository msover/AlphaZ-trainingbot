// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import frc.robot.util.Vector;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Telemetry;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.drive.SwerveConstants.TunerSwerveDrivetrain;

public class SwerveDrive implements Subsystem {
    private double MaxSpeed = SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.3).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity 0.75

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final TunerSwerveDrivetrain drivetrain = SwerveConstants.createDrivetrain();

    Vector targetVector = new Vector();
    Vector powerVector = new Vector();

   @Override
   public void initialize() {
      
   }

   @Override
   public void loop() {
      // Note that X is defined as forward according to WPILib convention,
      // and Y is defined as to the left according to WPILib convention.

      powerVector = new Vector(targetVector.getX(), targetVector.getY(), targetVector.getZ());

      drivetrain.setControl(
         drive.withVelocityX(powerVector.getX() * MaxSpeed*0.5)
            .withVelocityY(powerVector.getY() * MaxSpeed*0.5)
            .withRotationalRate(powerVector.getZ() * MaxAngularRate*1)
      );
      drivetrain.registerTelemetry(logger::telemeterize);
   }

   public void setTargetVector(Vector targetVector) {
      this.targetVector = targetVector;
   }
}

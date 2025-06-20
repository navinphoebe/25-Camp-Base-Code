// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class PathPlannerConfigurator extends SubsystemBase {
  /** Creates a new PathPlannerConfigurator. */

  public PathPlannerConfigurator() {
   try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> Robot.DRIVETRAIN_SUBSYSTEM.getPose(),   // Supplier of current robot pose
                Robot.DRIVETRAIN_SUBSYSTEM::resetOdometry,         // Consumer for seeding pose against auto
                () -> Robot.DRIVETRAIN_SUBSYSTEM.getSpeeds(), // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> Robot.DRIVETRAIN_SUBSYSTEM.driveSpeeds(speeds, feedforwards),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(15, 0, 0), 
                    // PID constants for rotation
                    new PIDConstants(15, 0, 0) 
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                Robot.DRIVETRAIN_SUBSYSTEM // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
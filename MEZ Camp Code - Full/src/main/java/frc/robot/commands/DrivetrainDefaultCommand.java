// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.util.constants.DrivetrainConstants;

public class DrivetrainDefaultCommand extends Command {
  /** Creates a new DrivetrainDefaultCommand. */
  public DrivetrainDefaultCommand() {
    addRequirements(Robot.DRIVETRAIN_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double controllerDirection = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 1 : -1;
    double x = Robot.DRIVER_CONTROLLER.getRawAxis(1) * controllerDirection;
    double y = Robot.DRIVER_CONTROLLER.getRawAxis(0) * controllerDirection;
    double r = Robot.DRIVER_CONTROLLER.getRawAxis(4) * -1;
    x = applyDeadband(x);
    y = applyDeadband(y);
    r = applyRotationalDeadband(r);
    
    if (Robot.DRIVER_CONTROLLER.rightTrigger().getAsBoolean()) {
      x *= DrivetrainConstants.CUT_POWER;
      y *= DrivetrainConstants.CUT_POWER;
      r *= DrivetrainConstants.CUT_POWER;
    }

    Robot.DRIVETRAIN_SUBSYSTEM.drive(x, y, r, true); 
  }

  private double applyDeadband(double x) {
    if (Math.abs(x) > DrivetrainConstants.CONTROLLER_DEADBAND_VALUE) {
      double percentage = (Math.abs(x) - DrivetrainConstants.CONTROLLER_DEADBAND_VALUE) / (1 - DrivetrainConstants.CONTROLLER_DEADBAND_VALUE);
      double sign = Math.copySign(1, x);
      double power = DrivetrainConstants.CONTROLLER_STATIC_FRICTION_BREAK * sign + (1 - DrivetrainConstants.CONTROLLER_STATIC_FRICTION_BREAK) * percentage * sign;
      return power;
    } else {
      return 0;
    }
  }

  private double applyRotationalDeadband(double x) {
    if (Math.abs(x) > DrivetrainConstants.CONTROLLER_DEADBAND_VALUE) {
      double percentage = (Math.abs(x) - DrivetrainConstants.CONTROLLER_DEADBAND_VALUE) / (1 - DrivetrainConstants.CONTROLLER_DEADBAND_VALUE);
      double sign = Math.copySign(1, x);
      double power = DrivetrainConstants.CONTROLLER_ROTATIONAL_STATIC_FRICTION_BREAK * sign + (1 - DrivetrainConstants.CONTROLLER_ROTATIONAL_STATIC_FRICTION_BREAK) * percentage * sign;
      return power;
    } else {
      return 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
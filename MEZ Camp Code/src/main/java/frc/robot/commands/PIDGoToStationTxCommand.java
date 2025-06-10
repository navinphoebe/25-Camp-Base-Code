// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.util.constants.DrivetrainConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PIDGoToStationTxCommand extends Command {

  private PIDController _controller = new PIDController(0, 0, 0);

  /** Creates a new PIDGoToStationTxCommand. */
  public PIDGoToStationTxCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.DRIVETRAIN_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tx = Robot.LIMELIGHT_SUBSYSTEM.getTX();
    double speed = _controller.calculate(tx);
    Robot.DRIVETRAIN_SUBSYSTEM.drive(speed, 0, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double tx = Robot.LIMELIGHT_SUBSYSTEM.getTX();
    return tx < DrivetrainConstants.TX_AUTO_DRIVE_DEADBAND;
  }
}

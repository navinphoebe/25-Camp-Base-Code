// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorGoToPositionCommand extends Command {
  /** Creates a new ElevatorGoToPositionCommand. */
  private double _position; 

  public ElevatorGoToPositionCommand(double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    _position = position;
    addRequirements(Robot.ELEVATOR_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.ELEVATOR_SUBSYSTEM.goToPositionMeters(_position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.ELEVATOR_SUBSYSTEM.goToPositionMeters(_position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; //Math.abs(_position - Robot.ELEVATOR_SUBSYSTEM.getPosition()) < ElevatorConstants.COMMAND_FINISH_DEADBAND;
  }
}

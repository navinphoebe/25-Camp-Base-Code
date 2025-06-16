// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DrivetrainDefaultCommand;
import frc.robot.commands.ElevatorDefaultCommand;
import frc.robot.commands.ElevatorGoToPositionCommand;
import frc.robot.commands.IntakeCoralCommand;
import frc.robot.commands.ShootCoralCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PathPlannerConfigurator;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.constants.DeviceConstants;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 * 
 * Note: In order to be able simulate code, we have the logged robot framework.
 */
public class Robot extends LoggedRobot {

  public static final IntakeSubsystem INTAKE_SUBSYSTEM = new IntakeSubsystem();
  public static final ShooterSubsystem SHOOTER_SUBSYSTEM = new ShooterSubsystem();
  public static final DrivetrainSubsystem DRIVETRAIN_SUBSYSTEM = new DrivetrainSubsystem();
  public static final PathPlannerConfigurator PATH_PLANNER_CONFIGURATOR = new PathPlannerConfigurator();
  public static final ElevatorSubsystem ELEVATOR_SUBSYSTEM = new ElevatorSubsystem();
  public static final CommandXboxController DRIVER_CONTROLLER = new CommandXboxController(DeviceConstants.DRIVER_CONTROLLER);
  public static final LimelightSubsystem LIMELIGHT_SUBSYSTEM = new LimelightSubsystem();


  private Command autoCommand;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
                    // be added.
  
    setButtonBindings();
  }

  public void setButtonBindings() {
    DRIVER_CONTROLLER.x().whileTrue(new IntakeCoralCommand());
    DRIVER_CONTROLLER.y().whileTrue(new ShootCoralCommand());
    DRIVER_CONTROLLER.a().onTrue(new ElevatorGoToPositionCommand(.6));
    DRIVER_CONTROLLER.b().onTrue(new ElevatorGoToPositionCommand(0));

    ELEVATOR_SUBSYSTEM.setDefaultCommand(new ElevatorDefaultCommand());
    DRIVETRAIN_SUBSYSTEM.setDefaultCommand(new DrivetrainDefaultCommand());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    autoCommand = getAutonomousCommand();
    // schedule the autonomous command (example)
    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  private Command getAutonomousCommand() {
     return new PathPlannerAuto("First Auto");
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (autoCommand != null) {
      autoCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}

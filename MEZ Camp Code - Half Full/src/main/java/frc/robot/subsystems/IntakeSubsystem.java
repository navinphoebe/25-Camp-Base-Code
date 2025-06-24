// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.constants.DeviceConstants;
import frc.robot.util.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final CANrange _intakeSensor = new CANrange(DeviceConstants.INTAKE_SENSOR);
  private final SparkMax _intakeMotorOne = new SparkMax(DeviceConstants.INTAKE_MOTOR, MotorType.kBrushless);
  private final SparkMax _intakeMotorTwo = new SparkMax(DeviceConstants.SHOOTER_MOTOR, MotorType.kBrushless);

  // Values for simulation - motor 1
  private final DCMotor _intakeGearboxOne = DCMotor.getNEO(1);
  private final SparkMaxSim _motorSimOne = new SparkMaxSim(_intakeMotorOne, _intakeGearboxOne);
  private final FlywheelSim _intakeSimOne = new FlywheelSim(LinearSystemId.createFlywheelSystem(_intakeGearboxOne,
      IntakeConstants.JKG_METERS_SQUARED, IntakeConstants.GEAR_RATIO), _intakeGearboxOne);
  private final LoggedDashboardChooser<Boolean> _intakeSensorChooser = new LoggedDashboardChooser<>("Intake Sensor");

  // Values for simulation - motor 2
  private final DCMotor _intakeGearboxTwo = DCMotor.getNEO(1);
  private final SparkMaxSim _motorSimTwo = new SparkMaxSim(_intakeMotorTwo, _intakeGearboxTwo);
  private final FlywheelSim _intakeSimTwo = new FlywheelSim(LinearSystemId.createFlywheelSystem(_intakeGearboxTwo,
      IntakeConstants.JKG_METERS_SQUARED, IntakeConstants.GEAR_RATIO), _intakeGearboxTwo);


  public IntakeSubsystem() {
    if (Robot.isSimulation()) {
      _intakeSensorChooser.addDefaultOption("No Coral", false);
      _intakeSensorChooser.addOption("Has Coral", true);
    }
  }

  public boolean hasCoral() {
    if (Robot.isSimulation()) {
      return _intakeSensorChooser.get();
    }
    //TODO: Write working logic
    return false;
  }

  public void updateSimulation(FlywheelSim intakeSim, SparkMaxSim motorSim) {
    intakeSim.setInput(motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    intakeSim.update(0.02);

    motorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute( // Motor Velocity, in RPM
            intakeSim.getAngularVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
        0.02); // Time interval, in Seconds

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(intakeSim.getCurrentDrawAmps()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.isSimulation()) {
      updateSimulation(_intakeSimOne, _motorSimOne);
      updateSimulation(_intakeSimTwo, _motorSimTwo);
    }
  }
}

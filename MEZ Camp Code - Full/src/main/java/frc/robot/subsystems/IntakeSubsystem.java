// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;
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
  private final SparkMax _intakeMotor = new SparkMax(DeviceConstants.INTAKE_MOTOR, MotorType.kBrushless);

  // Values for simulation
  private final DCMotor _intakeGearbox = DCMotor.getNEO(1);
  private final SparkMaxSim _motorSim = new SparkMaxSim(_intakeMotor, _intakeGearbox);
  private final FlywheelSim _intakeSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(_intakeGearbox,
      IntakeConstants.JKG_METERS_SQUARED, IntakeConstants.GEAR_RATIO), _intakeGearbox);
  private final LoggedDashboardChooser<Boolean> _intakeSensorChooser = new LoggedDashboardChooser<>("Intake Sensor");

  public IntakeSubsystem() {
    if (Robot.isSimulation()) {
      _intakeSensorChooser.addDefaultOption("No Coral", false);
      _intakeSensorChooser.addOption("Has Coral", true);
    }
  }

  @AutoLogOutput
  public boolean hasCoral() {
    if (Robot.isSimulation()) {
      return _intakeSensorChooser.get();
    }
    return _intakeSensor.getDistance().getValueAsDouble() < IntakeConstants.MAX_HAS_CORAL_DISTANCE;
  }

  @AutoLogOutput
  public double getSpeed() {
    return _intakeMotor.get();
  }

  public void intake() {
    _intakeMotor.set(IntakeConstants.INTAKE_SPEED);
  }

  public void reverse() {
    _intakeMotor.set(IntakeConstants.REVERSE_SPEED);
  }

  public void stop() {
    _intakeMotor.stopMotor();
  }

  public void updateSimulation() {
    _intakeSim.setInput(_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    _intakeSim.update(0.02);

    _motorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute( // Motor Velocity, in RPM
            _intakeSim.getAngularVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
        0.02); // Time interval, in Seconds

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(_intakeSim.getCurrentDrawAmps()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.isSimulation()) {
      updateSimulation();
    }
  }
}

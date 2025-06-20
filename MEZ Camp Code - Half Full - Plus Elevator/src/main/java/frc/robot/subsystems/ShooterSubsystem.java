// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import frc.robot.util.constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final SparkMax _shooterMotor = new SparkMax(DeviceConstants.SHOOTER_MOTOR, MotorType.kBrushless);

  // Values for simulation
  private final DCMotor _shooterGearbox = DCMotor.getNEO(1);
  private final SparkMaxSim _motorSim = new SparkMaxSim(_shooterMotor, _shooterGearbox);
  private final FlywheelSim _shooterSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(_shooterGearbox,
      ShooterConstants.JKG_METERS_SQUARED, ShooterConstants.GEAR_RATIO), _shooterGearbox);

  public ShooterSubsystem() {}

  public void updateSimulation() {
    _shooterSim.setInput(_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    _shooterSim.update(0.02);

    _motorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute( // Motor velocity, in RPM
            _shooterSim.getAngularVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
        0.02); // Time interval, in Seconds

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(_shooterSim.getCurrentDrawAmps()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.isSimulation()) {
      updateSimulation();
    }
  }
}

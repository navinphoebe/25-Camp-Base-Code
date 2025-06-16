// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.constants.DeviceConstants;
import frc.robot.util.constants.ElevatorConstants;
import frc.robot.util.simulation.MechanismSim;

public class ElevatorSubsystem extends SubsystemBase {
  private final SparkMax _elevatorMotor = new SparkMax(DeviceConstants.ELEVATOR_MOTOR, MotorType.kBrushless);
  private final SparkClosedLoopController _controller = _elevatorMotor.getClosedLoopController();
  private final SparkMaxConfig config = new SparkMaxConfig();

  private double _targetPosition;

  // Values for simulation
  private final DCMotor _elevatorGearbox = DCMotor.getNEO(1);
  private final SparkMaxSim _motorSim = new SparkMaxSim(_elevatorMotor, _elevatorGearbox);
  private final ElevatorSim _elevatorSim = new ElevatorSim(LinearSystemId.createElevatorSystem(_elevatorGearbox,
      ElevatorConstants.JKG_METERS_SQUARED, ElevatorConstants.RADIUS_DRUM, ElevatorConstants.GEAR_RATIO), _elevatorGearbox, ElevatorConstants.MIN_HEIGHT, ElevatorConstants.MAX_HEIGHT, ElevatorConstants.GRAVITY, ElevatorConstants.START_HEIGHT);
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    setUpConfig();
  }

  private void setUpConfig() {
  
    MAXMotionConfig maxMotion = new MAXMotionConfig()
    .maxVelocity(ElevatorConstants.MAX_VELOCITY)
    .maxAcceleration(ElevatorConstants.MAX_ACCELERATION)
    .allowedClosedLoopError(ElevatorConstants.ALLOWED_ERROR);

    config.closedLoop.apply(maxMotion);

    config.closedLoop
    .p(ElevatorConstants.kP, ClosedLoopSlot.kSlot0)
    .i(ElevatorConstants.kI, ClosedLoopSlot.kSlot0)
    .d(ElevatorConstants.kD, ClosedLoopSlot.kSlot0);

    _elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    _controller.setReference(0, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
  }

  public void goToPositionMeters(double position) {
    _targetPosition = position;
    _controller.setReference(position, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
  }

  @AutoLogOutput
  public double getSpeed() {
    return _elevatorMotor.get();
  }

  public void goToTargetPosition() {
    _controller.setReference(_targetPosition, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
  }

  @AutoLogOutput
  public double getTargetPosition() {
    return _targetPosition;
  }

  @AutoLogOutput
  public double getPosition() {
    return _elevatorMotor.getEncoder().getPosition();
  }

  public void updateSimulation() {
    _elevatorSim.setInput(_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

    _elevatorSim.update(0.02);

    _motorSim.iterate( _elevatorSim.getVelocityMetersPerSecond() * 60 / (2 * Math.PI * ElevatorConstants.RADIUS_DRUM),
        RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
        0.02); // Time interval, in Seconds

    // SimBattery estimates loaded battery voltages
    // This should include all motors being simulated
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(_elevatorSim.getCurrentDrawAmps()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.isSimulation()) {
      updateSimulation();
    }
    MechanismSim.updateElevatorHeight(getPosition());
  }
}

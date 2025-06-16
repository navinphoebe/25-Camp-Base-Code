/* 
  // TODO: Paste these class variables before the constructor and initalize a motor with the name: "_elevatorMotor"
  
  private final DCMotor _elevatorGearbox = DCMotor.getNEO(1);
  private final SparkMaxSim _motorSim = new SparkMaxSim(_elevatorMotor, _elevatorGearbox);
  private final ElevatorSim _elevatorSim = new ElevatorSim(LinearSystemId.createElevatorSystem(_elevatorGearbox,
      ElevatorConstants.JKG_METERS_SQUARED, ElevatorConstants.RADIUS_DRUM, ElevatorConstants.GEAR_RATIO), _elevatorGearbox, ElevatorConstants.MIN_HEIGHT, ElevatorConstants.MAX_HEIGHT, ElevatorConstants.GRAVITY, ElevatorConstants.START_HEIGHT);
 
 
  // TODO: Paste this method somewhere in the class

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

  // TODO: Put in periodic method

    if (Robot.isSimulation()) {
      updateSimulation();
    }
*/

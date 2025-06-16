/* 
  // TODO: Paste these class variables before the constructor and initalize a motor with the name: "_shooterMotor"

  private final DCMotor _shooterGearbox = DCMotor.getNEO(1);
  private final SparkMaxSim _motorSim = new SparkMaxSim(_shooterMotor, _shooterGearbox);
  private final FlywheelSim _shooterSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(_shooterGearbox,
      ShooterConstants.JKG_METERS_SQUARED, ShooterConstants.GEAR_RATIO), _shooterGearbox);

  // TODO: Paste this method somewhere in the class

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

  // TODO: Put in periodic method

    if (Robot.isSimulation()) {
      updateSimulation();
    }
*/
  

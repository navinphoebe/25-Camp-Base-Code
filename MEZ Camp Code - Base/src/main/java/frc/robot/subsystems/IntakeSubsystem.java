
/*
  // TODO: Paste these class variables before the constructor and initalize a motor with the name: "_intakeMotor"

  private final DCMotor _intakeGearbox = DCMotor.getNEO(1);
  private final SparkMaxSim _motorSim = new SparkMaxSim(_intakeMotor, _intakeGearbox);
  private final FlywheelSim _intakeSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(_intakeGearbox,
      IntakeConstants.JKG_METERS_SQUARED, IntakeConstants.GEAR_RATIO), _intakeGearbox);
  private final LoggedDashboardChooser<Boolean> _intakeSensorChooser = new LoggedDashboardChooser<>("Intake Sensor");


  // TODO: Put in subsystem constructor

    if (Robot.isSimulation()) {
      _intakeSensorChooser.addDefaultOption("No Coral", false);
      _intakeSensorChooser.addOption("Has Coral", true);
    }
  
  // TODO: Put this code in a hasCoral() method before the actual sensor return

    if (Robot.isSimulation()) {
      return _intakeSensorChooser.get();
    }
  
  // TODO: Paste this method somewhere in the class

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

  // TODO: Put in periodic method
   
    if (Robot.isSimulation()) {
      updateSimulation();
    }
*/


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.constants;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ElevatorConstants {

    public static final double RADIUS_DRUM = Units.inchesToMeters(2);

    public static final double LINEAR_MAX_VELOCITY = 10;
    public static final double LINEAR_MAX_ACCELERATION = 5;

    public static final double MAX_VELOCITY = ElevatorConstants.LINEAR_MAX_VELOCITY
            / (2 * Math.PI * ElevatorConstants.RADIUS_DRUM);
    public static final double MAX_ACCELERATION = ElevatorConstants.LINEAR_MAX_ACCELERATION
            / (2 * Math.PI * ElevatorConstants.RADIUS_DRUM);

    public static final double ALLOWED_ERROR = 0;

    public static final double kP = .5;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double GEAR_RATIO = 1;
    public static final double JKG_METERS_SQUARED = .1;

    public static final double MIN_HEIGHT = 0;
    public static final double MAX_HEIGHT = .7;
    public static final double START_HEIGHT = .5;
    public static final boolean GRAVITY = false;
    public static final double COMMAND_FINISH_DEADBAND = 0.01;
}

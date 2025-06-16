package frc.robot.util.constants;

import frc.robot.util.constants.Constants.DriveConstants;

public class DrivetrainConstants {
    public static final double CONTROLLER_DEADBAND_VALUE = .0144;
    public static final double CUT_POWER = .2;
    public static final double CONTROLLER_STATIC_FRICTION_BREAK = 0.07 / DriveConstants.kMaxSpeedMetersPerSecond;
    public static final double STATIC_FRICTION_BREAK = 0.15;
    public static final double ROTATION_STATIC_FRICTION_BREAK = 0.34;
    public static final double CONTROLLER_ROTATIONAL_STATIC_FRICTION_BREAK = 0.01 / DriveConstants.kMaxAngularSpeed;
    public static final double TX_AUTO_DRIVE_DEADBAND = 0.1;
}

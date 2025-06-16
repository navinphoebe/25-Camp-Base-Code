// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.drivetrain;

import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;

/** Add your docs here. */
public interface Gyro {

    public double getAngle(IMUAxis axis);
    public double getRate(IMUAxis axis);
    public void reset();
    public void update(double omegaRadiansPerSecond);
}

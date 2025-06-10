// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.drivetrain;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;

/** Add your docs here. */
public class ADIS implements Gyro {
    private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

    public ADIS() {}

    public double getAngle(IMUAxis axis) {
        return m_gyro.getAngle(axis);
    }

    public double getRate(IMUAxis axis) {
        return m_gyro.getRate(axis);
    }

    public void reset() {
        m_gyro.reset();
    }

    // Will only be called in simulation
    public void update(double degrees) {}
}

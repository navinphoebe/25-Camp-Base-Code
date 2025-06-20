// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.drivetrain;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class GyroSim implements Gyro {

    private double _rate = 0; 

    public Timer rotateTimer = new Timer();

    public double _rotationDegrees;

    public GyroSim() {
        rotateTimer.start();
        _rotationDegrees = 0;
    }

    public double getAngle(IMUAxis axis) {
        return _rotationDegrees;
    }
    public double getRate(IMUAxis axis) {
        return _rate;
    }

    public void reset() {
        _rotationDegrees = 0;
    }


    public void update(double omegaRadiansPerSecond) {
        double time = rotateTimer.get();
        rotateTimer.reset();
        double value = omegaRadiansPerSecond * time;
        _rotationDegrees += Units.radiansToDegrees(value);
        _rate = Units.radiansToDegrees(omegaRadiansPerSecond);
    }
}
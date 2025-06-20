// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class SwerveModuleSim implements SwerveModule {

    public double _velocity = 0;
    public Rotation2d _rotation = new Rotation2d(0);
    public double _distance = 0;

    public Timer timer = new Timer();

    public SwerveModuleSim() {
        timer.start();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(_velocity,
        _rotation);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(_distance, _rotation);
    }

    public void resetEncoders() {
        _rotation = new Rotation2d(0);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        double time = timer.get();
        timer.reset();
        double value = desiredState.speedMetersPerSecond * time;
        _distance += value;
        _rotation = desiredState.angle;
        _velocity = desiredState.speedMetersPerSecond;
    }
}
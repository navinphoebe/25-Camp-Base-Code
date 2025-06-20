// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.simulation;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

/** Add your docs here. */
public class MechanismSim {

    //elevator variables
    private static Pose3d[] elevatorPoses = {new Pose3d(), new Pose3d()};
    private static final double HEIGHT_MAX = .35; // Each elevator portion can only extend .35 meters each.
    private static final double HEIGHT_MIN = 0;
    private static double elevatorSingleStageHeight = 0;
    private static double _elevatorTotalHeight = 0;

    // elevator methods
    public static void updateElevatorPoseChange(double changeTotalElevatorHeight) {
        _elevatorTotalHeight += changeTotalElevatorHeight;
        updateElevatorHeight(_elevatorTotalHeight);
    }

    public static void updateElevatorHeight(double elevatorTotalHeight) {
        elevatorSingleStageHeight = elevatorTotalHeight / 2;

        elevatorSingleStageHeight = Math.min(elevatorSingleStageHeight, HEIGHT_MAX);
        elevatorSingleStageHeight = Math.max(elevatorSingleStageHeight, HEIGHT_MIN);
    
        _elevatorTotalHeight = elevatorSingleStageHeight * 2;

        elevatorPoses[0] = new Pose3d(0, 0, elevatorSingleStageHeight, new Rotation3d()); // update
        elevatorPoses[1] = new Pose3d(elevatorPoses[0].getX(), elevatorPoses[0].getY(), elevatorPoses[0].getZ() + elevatorSingleStageHeight, elevatorPoses[0].getRotation());
        Logger.recordOutput("MechSim/Elevator", elevatorPoses);
    }

    @AutoLogOutput
    public static double getElevatorHeight() {
        return elevatorSingleStageHeight * 2;
    }

}
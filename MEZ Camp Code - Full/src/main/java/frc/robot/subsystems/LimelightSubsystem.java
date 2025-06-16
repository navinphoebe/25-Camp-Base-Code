// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {

  private static final String NAME = "";

  private double tx = LimelightHelpers.getTX(NAME); // Horizontal offset from crosshair to target in degrees
  private double ty = LimelightHelpers.getTY(NAME); // Vertical offset from crosshair to target in degrees
  private double ta = LimelightHelpers.getTA(NAME); // Target area (0% to 100% of image)
  private boolean hasTarget = LimelightHelpers.getTV(NAME); // Do you have a valid target?F

  /** Creates a new VisionSubsystem. */
  public LimelightSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    hasTarget = LimelightHelpers.getTV(NAME); // Do you have a valid target?
    if (hasTarget) {
      tx = LimelightHelpers.getTX(NAME); // Horizontal offset from crosshair to target in degrees
      ty = LimelightHelpers.getTY(NAME); // Vertical offset from crosshair to target in degrees
      ta = LimelightHelpers.getTA(NAME); // Target area (0% to 100% of image)
    }
  }

  public double getTX() {
    return tx;
  }

  public double getTY() {
    return ty;
  }

  public double getTA() {
    return ta;
  }

  public boolean hasTarget() {
    return hasTarget;
  }

}

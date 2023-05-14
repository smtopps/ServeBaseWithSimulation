// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pigeon2Subsystem extends SubsystemBase {

  private final Pigeon2 pigeon2 = new Pigeon2(Constants.DRIVETRAIN_PIGEON_ID, Constants.DRIVETRAIN_CANBUS);

  public Pigeon2Subsystem() {
    pigeon2.configFactoryDefault();
    pigeon2.configMountPose(AxisDirection.PositiveY, AxisDirection.PositiveZ);
  }

  @Override
  public void periodic() {}

  /**
   * zeroGyroscope
   * Sets the angle to 0 in degrees
   */
  public void zeroGyroscope() {
    pigeon2.setYaw(0);
  }

  /**
   * getGyroRotation - this is the Yaw value (rotate around...)
   * @return Rotation2d
   */
  public Rotation2d getGyroRotation() {
    return Rotation2d.fromDegrees(pigeon2.getYaw());
  }

  /**
   * getPigeonPitch - this is the pitch value (tilt up or down)
   * @return double
   */
  public double getPigeonPitch(){
    return pigeon2.getPitch();
  }

  /**
   * getPigeonRoll - this is the roll value (tilt left or right)
   * @return double
   */
  public double getPigeonRoll(){
    return pigeon2.getRoll();
  }

  /**
   * getPigeonYaw - this is the Yaw value (rotate around...)
   * @return double
   */
  public double getPigeonYaw(){
    return pigeon2.getYaw();
  }
}

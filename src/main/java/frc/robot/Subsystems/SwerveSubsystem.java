// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.SwerveModuleConstants;
import frc.robot.GlobalVariables;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {

  public SwerveModule[] swerveModules;

  public SwerveSubsystem() {
    swerveModules = new SwerveModule[] {
      new SwerveModule(0, new SwerveModuleConstants(
      SwerveConstants.FRONT_LEFT_DRIVE_MOTOR, 
      SwerveConstants.FRONT_LEFT_STEER_MOTOR, 
      SwerveConstants.FRONT_LEFT_STEER_ENCODER, 
      SwerveConstants.FRONT_LEFT_STEER_OFFSET)),

      new SwerveModule(1, new SwerveModuleConstants(
      SwerveConstants.FRONT_RIGHT_DRIVE_MOTOR, 
      SwerveConstants.FRONT_RIGHT_STEER_MOTOR, 
      SwerveConstants.FRONT_RIGHT_STEER_ENCODER, 
      SwerveConstants.FRONT_RIGHT_STEER_OFFSET)),

      new SwerveModule(2, new SwerveModuleConstants(
      SwerveConstants.BACK_LEFT_DRIVE_MOTOR, 
      SwerveConstants.BACK_LEFT_STEER_MOTOR, 
      SwerveConstants.BACK_LEFT_STEER_ENCODER, 
      SwerveConstants.BACK_LEFT_STEER_OFFSET)),

      new SwerveModule(3, new SwerveModuleConstants(
      SwerveConstants.BACK_RIGHT_DRIVE_MOTOR, 
      SwerveConstants.BACK_RIGHT_STEER_MOTOR, 
      SwerveConstants.BACK_RIGHT_STEER_ENCODER, 
      SwerveConstants.BACK_RIGHT_STEER_OFFSET))
    };

    if(DriverStation.getAlliance() == Alliance.Blue) {
      GlobalVariables.isBlue = true;
    }else{
      GlobalVariables.isBlue = false;
    }
  }

  @Override
  public void periodic() {}

  /**
   * Main controlling method for driving swerve based on desired speed of drivetrian
   * @param chassisSpeeds Desired speed of drivetrain
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] states = SwerveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(states);
  }

  /**
   * Set the desired state of all the modules
   * @param states Desired module states
   */
  public void setModuleStates(SwerveModuleState[] states){
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND);
    swerveModules[0].setDesiredState(states[0], true);
    swerveModules[1].setDesiredState(states[1], true);
    swerveModules[2].setDesiredState(states[2], true); 
    swerveModules[3].setDesiredState(states[3], true); 
  }

  /**
   * Stops all movement for swerve modules
   */
  public void stop(){
    swerveModules[0].stop();
    swerveModules[1].stop();
    swerveModules[2].stop();
    swerveModules[3].stop();
  }

  /**
   * Rotates modules in an X shape which makes it hard to push
   */
  public void lock(){
    swerveModules[0].setDesiredStateAbs(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)), false);
    swerveModules[1].setDesiredStateAbs(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)), false);
    swerveModules[2].setDesiredStateAbs(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)), false);
    swerveModules[3].setDesiredStateAbs(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)), false);
  }

  /**
   * Get the position of all the modules including distance and angle of each
   * @return Position of all the modules as an array
   */
  public SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {
      swerveModules[0].getPosition(), swerveModules[1].getPosition(), swerveModules[2].getPosition(), swerveModules[3].getPosition()
    };
  }

  /**
   * Get the state of all the modules including velocity and angle of each
   * @return The state of all the modules as an array
   */
  public SwerveModuleState[] getStates() {
    return new SwerveModuleState[] {
      swerveModules[0].getState(), swerveModules[1].getState(), swerveModules[2].getState(), swerveModules[3].getState()
    };
  }

  /**
   * @return the current speed of the robot in whatever direction it is traveling
   */
  public double getCurrentChassisSpeeds() {
    ChassisSpeeds currentSpeeds = SwerveConstants.KINEMATICS.toChassisSpeeds(this.getStates());
    double linearVelocity = Math.sqrt((currentSpeeds.vxMetersPerSecond * currentSpeeds.vxMetersPerSecond) + (currentSpeeds.vyMetersPerSecond * currentSpeeds.vyMetersPerSecond));
    return linearVelocity;
  }

  /**
   * @param currentPose of the robot from the pose estimator
   * @return the current direction the robot is traveling in
   */
  public Rotation2d getCurrentChassisHeading(Pose2d currentPose) {
    ChassisSpeeds currentSpeeds = SwerveConstants.KINEMATICS.toChassisSpeeds(this.getStates());
    Rotation2d robotHeading = new Rotation2d(Math.atan2(currentSpeeds.vyMetersPerSecond, currentSpeeds.vxMetersPerSecond));
    Rotation2d currentHeading = robotHeading.plus(currentPose.getRotation());
    return currentHeading;
  }
}

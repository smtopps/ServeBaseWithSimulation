// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class PoseEstimator extends SubsystemBase {

  private final SwerveSubsystem swerveSubsystem;
  private final Pigeon2Subsystem pigeon2Subsystem;
  

  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much you trust your various sensors. 
  // Smaller numbers will cause the filter to "trust" the estimate from that particular component more than the others. 
  // This in turn means the particualr component will have a stronger influence on the final pose estimate.
  private final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1); //was 0.05, 0.05, deg to rad 5
  private final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.9, 0.9, 0.9); //was 0.02, 0.02, 5
  private static SwerveDrivePoseEstimator poseEstimator;
  
  private final Field2d field2d = new Field2d();

  public PoseEstimator(SwerveSubsystem swerveSubsystem, Pigeon2Subsystem pigeon2Subsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.pigeon2Subsystem = pigeon2Subsystem;

    poseEstimator = new SwerveDrivePoseEstimator(
      SwerveConstants.KINEMATICS, 
      pigeon2Subsystem.getGyroRotation(), 
      swerveSubsystem.getPositions(), 
      new Pose2d(new Translation2d(0, 0), new Rotation2d(0.0)), 
      stateStdDevs, 
      visionMeasurementStdDevs);

    SmartDashboard.putData("Field", field2d);
  }

  @Override
  public void periodic() {
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), pigeon2Subsystem.getGyroRotation(), swerveSubsystem.getPositions());
    field2d.setRobotPose(poseEstimator.getEstimatedPosition());
  }

  /**
   * Get the current pose of the robot using the pose estimator
   * @return Pose2d representing the current estimated X, Y, and Theta of the robot
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Get the current X position of the robot using the pose estimator
   * @return double representing the current estimated X of the robot
   */
  public double getPoseX() {
    return poseEstimator.getEstimatedPosition().getX();
  }

  /**
   * Get the current Y position of the robot using the pose estimator
   * @return double representing the current estimated Y of the robot
   */
  public double getPoseY() {
    return poseEstimator.getEstimatedPosition().getY();
  }

  /**
   * Get the current Theta position of the robot using the pose estimator
   * @return double representing the current estimated Theta of the robot
   */
  public double getPoseTheta() {
    return poseEstimator.getEstimatedPosition().getRotation().getDegrees();
  }

  /**
   * Get the current Theta position of the robot using the pose estimator
   * @return Rotation2d representing the current estimated Theta of the robot
   */
  public Rotation2d getPoseRotation() {
    return poseEstimator.getEstimatedPosition().getRotation();
  }

  /**
   * Sets the pose of the pose estimator
   * @param pose to set the pose estimator to
   */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(pigeon2Subsystem.getGyroRotation(), swerveSubsystem.getPositions(), pose);
  }

  /**
   * Draws a trajectory on the field2d object to view on shuffleboard
   * @param trajectory to be drawn on the field2d object
   */
  public void setTrajectoryField2d(Trajectory trajectory) {
    field2d.getObject("traj").setTrajectory(trajectory);
  }
}

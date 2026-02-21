// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.CanIdConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.OIConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.studica.frc.AHRS;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      CanIdConstants.kFrontLeftDrivingCanId,
      CanIdConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      CanIdConstants.kFrontRightDrivingCanId,
      CanIdConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      CanIdConstants.kRearLeftDrivingCanId,
      CanIdConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      CanIdConstants.kRearRightDrivingCanId,
      CanIdConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  // private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
  
      double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
      double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
      double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;
  
      var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
          fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                  Rotation2d.fromDegrees(m_gyro.getAngle()))
              : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
      
      SwerveDriveKinematics.desaturateWheelSpeeds(
          swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
  
      // If speed is zero, keep the modules at their current angles
      if (xSpeed == 0 && ySpeed == 0 && rot == 0) {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, m_frontLeft.getState().angle));
        m_frontRight.setDesiredState(new SwerveModuleState(0, m_frontRight.getState().angle));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, m_rearLeft.getState().angle));
        m_rearRight.setDesiredState(new SwerveModuleState(0, m_rearRight.getState().angle));
      } else {
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
      }
    }
  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  public void driveWithChassisSpeeds(ChassisSpeeds speeds) {
    // Convert ChassisSpeeds to SwerveModuleStates
    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

    // Normalize wheel speeds to be within max speed limits
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    // Set the desired states to each swerve module
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public Command driveCommand(
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotSupplier,
      BooleanSupplier fieldRelativeSupplier) {

    return this.run(() -> {
      // We stole the deadband logic from DriveCommand and put it here
      double xSpeed = -MathUtil.applyDeadband(xSupplier.getAsDouble(), OIConstants.kDriveDeadband);
      double ySpeed = -MathUtil.applyDeadband(ySupplier.getAsDouble(), OIConstants.kDriveDeadband);
      double rot = -MathUtil.applyDeadband(rotSupplier.getAsDouble(), OIConstants.kDriveDeadband);

      // Then we just call the existing drive method
      this.drive(xSpeed, ySpeed, rot, fieldRelativeSupplier.getAsBoolean());
    })
        .withName("DefaultDrive")
        .finallyDo(() -> this.drive(0, 0, 0, false));
  }

  public SparkMax getRightFrontDrive() {
    return this.m_frontRight.getDrive();
  }

  public SparkMax getRightFrontTurn() {
    return this.m_frontRight.getTurn();
  }

  public SparkMax getLeftFrontDrive() {
    return this.m_frontLeft.getDrive();
  }

  public SparkMax getLeftFrontTurn() {
    return this.m_frontLeft.getTurn();
  }

  public SparkMax getRightRearDrive() {
    return this.m_rearRight.getDrive();
  }

  public SparkMax getRightRearTurn() {
    return this.m_rearRight.getTurn();
  }

  public SparkMax getLeftRearDrive() {
    return this.m_rearLeft.getDrive();
  }

  public SparkMax getLeftRearTurn() {
    return this.m_rearLeft.getTurn();
  }
}

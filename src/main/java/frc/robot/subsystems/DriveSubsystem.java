// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.CanIdConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.OIConstants;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.Telemetry;
import frc.robot.utils.VisionUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import frc.robot.configs.DriveConfigs;
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
  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

  // Pose estimator for tracking robot pose, integrates wheel odometry and vision
  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      },
      new Pose2d());

  private final LoggedTunableNumber driveP = new LoggedTunableNumber("Drive/DriveP", DriveConstants.kDriveP);
  private final LoggedTunableNumber driveS = new LoggedTunableNumber("Drive/DriveS", DriveConstants.kDriveS);
  private final LoggedTunableNumber driveV = new LoggedTunableNumber("Drive/DriveV", DriveConstants.kDriveV);

  private final LoggedTunableNumber turnP = new LoggedTunableNumber("Drive/TurnP", DriveConstants.kTurnP);
  private final LoggedTunableNumber turnS = new LoggedTunableNumber("Drive/TurnS", DriveConstants.kTurnS);
  private final LoggedTunableNumber turnV = new LoggedTunableNumber("Drive/TurnV", DriveConstants.kTurnV);

  private final LoggedTunableNumber maxSpeed = new LoggedTunableNumber("Drive/MaxSpeed",
      DriveConstants.kMaxSpeedMetersPerSecond);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      config = null;
    }

    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getChassisSpeeds,
        (speeds, feedforwards) -> driveWithChassisSpeeds(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID — tune these
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID — tune these
        ),
        config,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
  }

  @Override
  public void periodic() {

    if (GlobalConstants.kTuningMode) {
      // Update tuning values if they changed
      LoggedTunableNumber.ifChanged(hashCode(), () -> {
        // Create new configs based on the base config but with the new P/FF values
        SparkMaxConfig driveConfig = new SparkMaxConfig().apply(DriveConfigs.MAXSwerveModule.drivingConfig);
        driveConfig.closedLoop.p(driveP.get());
        driveConfig.closedLoop.feedForward.kS(driveS.get());
        driveConfig.closedLoop.feedForward.kV(driveV.get());

        SparkMaxConfig turnConfig = new SparkMaxConfig().apply(DriveConfigs.MAXSwerveModule.turningConfig);
        turnConfig.closedLoop.p(turnP.get());
        turnConfig.closedLoop.feedForward.kS(turnS.get());
        turnConfig.closedLoop.feedForward.kV(turnV.get());

        // Apply to all 4 modules
        SparkMax[] driveMotors = { m_frontLeft.getDrive(), m_frontRight.getDrive(), m_rearLeft.getDrive(),
            m_rearRight.getDrive() };
        SparkMax[] turnMotors = { m_frontLeft.getTurn(), m_frontRight.getTurn(), m_rearLeft.getTurn(),
            m_rearRight.getTurn() };

        for (SparkMax motor : driveMotors) {
          motor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        }
        for (SparkMax motor : turnMotors) {
          motor.configure(turnConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        }
      }, driveP, driveS, driveV, turnP, turnS, turnV);
    }
    // Update the odometry in the periodic block
    m_poseEstimator.update(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    // Add vision measurements if a target is visible
    var table = NetworkTableInstance.getDefault().getTable("limelight");
    if (table.getEntry("tv").getDouble(0) == 1.0) {
      // Subtract latency (tl + cl) from current time. Table values are in
      // milliseconds, so divide by 1000.
      double latency = (table.getEntry("tl").getDouble(0) + table.getEntry("cl").getDouble(0)) / 1000.0;
      double timestamp = Timer.getFPGATimestamp() - latency;

      m_poseEstimator.addVisionMeasurement(VisionUtils.getBotPose(), timestamp);
    }

    // --- Telemetry ---
    Pose2d pose = getPose();
    Telemetry.putDebugNumber("Drive/Robot X", pose.getX());
    Telemetry.putDebugNumber("Drive/Robot Y", pose.getY());
    Telemetry.putNumber("Drive/Robot Heading", pose.getRotation().getDegrees());

    ChassisSpeeds speeds = getChassisSpeeds();
    Telemetry.putDebugNumber("Drive/VX", speeds.vxMetersPerSecond);
    Telemetry.putDebugNumber("Drive/VY", speeds.vyMetersPerSecond);
    Telemetry.putDebugNumber("Drive/Omega", speeds.omegaRadiansPerSecond);

    // Debug Module States
    Telemetry.putDebugNumber("Drive/FL Velocity", m_frontLeft.getState().speedMetersPerSecond);
    Telemetry.putDebugNumber("Drive/FR Velocity", m_frontRight.getState().speedMetersPerSecond);
    Telemetry.putDebugNumber("Drive/RL Velocity", m_rearLeft.getState().speedMetersPerSecond);
    Telemetry.putDebugNumber("Drive/RR Velocity", m_rearRight.getState().speedMetersPerSecond);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
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

    double xSpeedDelivered = xSpeed * maxSpeed.get();
    double ySpeedDelivered = ySpeed * maxSpeed.get();
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, maxSpeed.get());

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
        desiredStates, maxSpeed.get());
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
  private void zeroHeading() {
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
        .finallyDo(() -> this.drive(0, 0, 0, fieldRelativeSupplier.getAsBoolean()));
  }

  public Command orient() {
    return this.run(() -> zeroHeading())
        .withName("OrientBot");
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

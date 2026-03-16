// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.configs.DriveConfigs;

public class MAXSwerveModule {
  private final SparkMax m_DrivingMotor;
  private final SparkMax m_TurningMotor;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  // Simulation support
  private double m_simPosition = 0;
  private double m_simVelocity = 0;
  private Rotation2d m_simAngle = new Rotation2d();

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_DrivingMotor = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_TurningMotor = new SparkMax(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_DrivingMotor.getEncoder();
    m_turningEncoder = m_TurningMotor.getAbsoluteEncoder();

    m_drivingClosedLoopController = m_DrivingMotor.getClosedLoopController();
    m_turningClosedLoopController = m_TurningMotor.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_DrivingMotor.configure(DriveConfigs.MAXSwerveModule.drivingConfig, ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);
    m_TurningMotor.configure(DriveConfigs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    if (RobotBase.isSimulation()) {
      return new SwerveModuleState(m_simVelocity, m_simAngle);
    }
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  public SwerveModulePosition getPosition() {
    if (RobotBase.isSimulation()) {
      return new SwerveModulePosition(m_simPosition, m_simAngle);
    }
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // 1. Optimize the state based on the CURRENT (offset-applied) encoder reading
    Rotation2d currentRotation = new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset);
    
    SwerveModuleState correctedDesiredState = new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
    correctedDesiredState.optimize(currentRotation);

    // 2. Command the SPARK MAX controllers
    // For the motor, we ADD the offset back so the pod points where the chassis thinks it should
    m_drivingClosedLoopController.setSetpoint(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningClosedLoopController.setSetpoint(correctedDesiredState.angle.getRadians() + m_chassisAngularOffset, ControlType.kPosition);

    m_desiredState = correctedDesiredState;
  }

  public void updateSimulation(double dtSeconds) {
    m_simVelocity = m_desiredState.speedMetersPerSecond;
    m_simPosition += m_simVelocity * dtSeconds;
    m_simAngle = m_desiredState.angle;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  public SparkMax getDrive() {
    return this.m_DrivingMotor;
  }

  public SparkMax getTurn() {
    return this.m_TurningMotor;
  }
}

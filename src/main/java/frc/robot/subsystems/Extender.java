// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.configs.ExtenderConfigs;
import frc.robot.constants.CanIdConstants;
import frc.robot.constants.ExtenderConstants;
import frc.robot.constants.GlobalConstants;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.util.Units;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.Telemetry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Extender extends SubsystemBase {
  private final SparkMax m_LeaderMotor;
  private final SparkMax m_FollowMotor;

  private final SparkClosedLoopController m_LeaderController;
  private final SparkClosedLoopController m_FollowController;

  private final RelativeEncoder m_LeaderEncoder;
  private final RelativeEncoder m_FollowEncoder;

  private double m_TargetPOS = 0.0;
  private boolean m_isFaulted = false;

  private final LoggedTunableNumber kP_Tuning = new LoggedTunableNumber("Extender/kP", 0.85);
  private final LoggedTunableNumber kS_Tuning = new LoggedTunableNumber("Extender/kS", ExtenderConstants.kS);
  private final LoggedTunableNumber kV_Tuning = new LoggedTunableNumber("Extender/kV", ExtenderConstants.kV);
  private final LoggedTunableNumber kA_Tuning = new LoggedTunableNumber("Extender/kA", ExtenderConstants.kA);

  private final LoggedTunableNumber kCruiseVelocity_Tuning = new LoggedTunableNumber("Extender/CruiseVelocity",
      ExtenderConstants.kCruiseVelocity);
  private final LoggedTunableNumber kMaxAccel_Tuning = new LoggedTunableNumber("Extender/MaxAccel",
      ExtenderConstants.kMaxAccel);

  private final LoggedTunableNumber maxExtensionInches = new LoggedTunableNumber("Extender/MaxExtensionInches",
      Units.metersToInches(ExtenderConstants.kExtendOutTarget));
  private final LoggedTunableNumber partialExtendInches = new LoggedTunableNumber("Extender/PartialExtendInches",
      Units.metersToInches(ExtenderConstants.kExtendPartialTarget));
  private final LoggedTunableNumber launcherExtendInches = new LoggedTunableNumber("Extender/LauncherExtendInches",
      Units.metersToInches(ExtenderConstants.kExtendLaunchTarget));

  /** Creates a new Extender. */
  public Extender() {

    this.m_LeaderMotor = new SparkMax(CanIdConstants.kExtenderMotor1, SparkLowLevel.MotorType.kBrushless);
    this.m_FollowMotor = new SparkMax(CanIdConstants.kExtenderMotor2, SparkLowLevel.MotorType.kBrushless);

    this.m_LeaderController = this.m_LeaderMotor.getClosedLoopController();
    this.m_FollowController = this.m_FollowMotor.getClosedLoopController();

    this.m_LeaderEncoder = this.m_LeaderMotor.getEncoder();
    this.m_FollowEncoder = this.m_FollowMotor.getEncoder();

    // Configure motor controllers (no encoder config needed for SparkMax)
    SparkMaxConfig leaderConfig = new SparkMaxConfig().apply(ExtenderConfigs.config);
    leaderConfig.inverted(false);
    this.m_LeaderMotor.configure(leaderConfig, ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

    SparkMaxConfig followerConfig = new SparkMaxConfig().apply(ExtenderConfigs.config);
    followerConfig.inverted(true);
    // followerConfig.follow(this.m_LeaderMotor, true);
    this.m_FollowMotor.configure(followerConfig, ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

    this.m_LeaderEncoder.setPosition(0);
    this.m_FollowEncoder.setPosition(0);
  }

  @Override
  public void periodic() {

    if (GlobalConstants.kTuningMode) {
      // Update tuning values if they changed on the dashboard
      LoggedTunableNumber.ifChanged(hashCode(), () -> {
        SparkMaxConfig config = new SparkMaxConfig().apply(ExtenderConfigs.config);
        config.closedLoop.p(kP_Tuning.get());

        // Update FeedForward
        config.closedLoop.feedForward.kS(kS_Tuning.get());
        config.closedLoop.feedForward.kV(kV_Tuning.get());
        config.closedLoop.feedForward.kA(kA_Tuning.get());

        config.closedLoop.maxMotion.cruiseVelocity(kCruiseVelocity_Tuning.get());
        config.closedLoop.maxMotion.maxAcceleration(kMaxAccel_Tuning.get());

        m_LeaderMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        m_FollowMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      }, kP_Tuning, kS_Tuning, kV_Tuning, kA_Tuning, kCruiseVelocity_Tuning, kMaxAccel_Tuning);
    }

    double leaderPos = this.m_LeaderEncoder.getPosition();
    double followPos = this.m_FollowEncoder.getPosition();

    Telemetry.putBoolean("Extender/Faulted", !m_isFaulted);
    Telemetry.putDebugNumber("Extender/Leader Output", m_LeaderMotor.getAppliedOutput());
    Telemetry.putDebugNumber("Extender/Follow Output", m_FollowMotor.getAppliedOutput());
    Telemetry.putDebugNumber("Extender/Leader Current", m_LeaderMotor.getOutputCurrent()); // fix the bug too
    Telemetry.putDebugNumber("Extender/Leader Velocity", m_LeaderEncoder.getVelocity());
    Telemetry.putDebugNumber("Extender/Follow Velocity", m_FollowEncoder.getVelocity());
    Telemetry.putDebugNumber("Extender/Leader POS", Units.metersToInches(leaderPos));
    Telemetry.putDebugNumber("Extender/Follow POS", Units.metersToInches(followPos));
    Telemetry.putDebugNumber("Extender/Target POS", Units.metersToInches(this.m_TargetPOS));
    Telemetry.putDebugNumber("Extender/Leader Current", m_FollowMotor.getOutputCurrent());
    Telemetry.putDebugNumber("Extender/Follow Current", m_FollowMotor.getOutputCurrent());
    Telemetry.putDebugNumber("Extender/Position Tolerace", ExtenderConstants.kPositionTolerance);
    Telemetry.putDebugNumber("Extender/CruiseVelocity", ExtenderConstants.kCruiseVelocity);
    Telemetry.putDebugNumber("Extender/MaxAccel", ExtenderConstants.kMaxAccel);
    Telemetry.putDebugNumber("Extender/Leader Faults", m_LeaderMotor.getFaults().rawBits);
    Telemetry.putDebugNumber("Extender/Follow Faults", m_FollowMotor.getFaults().rawBits);
    Telemetry.putDebugNumber("Extender/Leader POS Raw", m_LeaderEncoder.getPosition());
    Telemetry.putString("Extender/Leader Last Error", m_LeaderMotor.getLastError().toString());

    if (m_isFaulted) {
      stopMotors(); // Keep them stopped if we are already in a fault state
      return;
    }

    /*/ High current + low velocity = stall
    boolean leaderStalled = m_LeaderMotor.getOutputCurrent() > ExtenderConstants.kCurrentLimit
        && Math.abs(m_LeaderEncoder.getVelocity()) < 0.01;
    boolean followStalled = m_FollowMotor.getOutputCurrent() > ExtenderConstants.kCurrentLimit
        && Math.abs(m_FollowEncoder.getVelocity()) < 0.01;

    if (leaderStalled || followStalled) {
      m_isFaulted = true;
      stopMotors();
    }*/

    if (Math.abs(leaderPos - followPos) > ExtenderConstants.kMaxPositionDifference) {
      m_isFaulted = true; // Trip the software breaker
      stopMotors();
    }
  }

  private void stopMotors() {
    this.m_LeaderMotor.stopMotor();
    this.m_FollowMotor.stopMotor();
  }

  private Command goToPosition(double position) {
    return this.run(() -> {
      System.out.println("goToPosition running, target: " + position + " faulted: " + m_isFaulted);

      if (!m_isFaulted) {
        this.m_LeaderController.setSetpoint(position, ControlType.kMAXMotionPositionControl);
        this.m_FollowController.setSetpoint(position, ControlType.kMAXMotionPositionControl);
        // this.m_LeaderController.setSetpoint(position, ControlType.kPosition);
        this.m_TargetPOS = position;
      } else if (position == 0.0) {
        this.m_LeaderController.setSetpoint(position, ControlType.kMAXMotionPositionControl);
        this.m_FollowController.setSetpoint(position, ControlType.kMAXMotionPositionControl);
        // this.m_LeaderController.setSetpoint(position, ControlType.kPosition);
        this.m_TargetPOS = position;
      }
    })
        .until(() -> m_isFaulted ||
            (Math.abs(m_LeaderEncoder.getPosition() - position) < ExtenderConstants.kPositionTolerance
                && Math.abs(m_FollowEncoder.getPosition() - position) < ExtenderConstants.kPositionTolerance))
        .withTimeout(3.0)
        .finallyDo((interrupted) -> stopMotors())
        .withName("ExtenderTo" + position);
  }

  public Command stow() {
    return goToPosition(0.0)
        .withName("ExtenderIn");
  }

  public Command fullExtend() {
    return goToPosition(Units.inchesToMeters(maxExtensionInches.get()))
        .withName("ExtenderOut");
  }

  public Command partialExtend() {
    return goToPosition(Units.inchesToMeters(partialExtendInches.get()))
        .withName("ExtenderPartial");
  }

  public Command launchExtend() {
    return goToPosition(Units.inchesToMeters(launcherExtendInches.get()))
        .withName("ExtenderLaunch");
  }

  public Command home() {
    return this.run(() -> {
      // Apply a gentle negative voltage to move toward the hard stop
      // Note: If 'stow' is 0.0, you likely need a negative value (e.g., -0.2)
      // to drive into the physical bottom.
      this.m_LeaderMotor.set(-0.2);
      this.m_FollowMotor.set(-0.2);
    })
        // Trigger when current spikes (30A is a safe starting point for a NEO)
        .until(() -> m_LeaderMotor.getOutputCurrent() > GlobalConstants.kLowCurrentLimit
            && m_FollowMotor.getOutputCurrent() > GlobalConstants.kLowCurrentLimit)
        .finallyDo((interrupted) -> {
          stopMotors();
          if (!interrupted) {
            // Reset both encoders to 0.0 only if we reached the stop (not cancelled)
            this.m_TargetPOS = 0.0;
            this.m_isFaulted = false;
            this.m_LeaderEncoder.setPosition(0.0);
            this.m_FollowEncoder.setPosition(0.0);
          }
        })
        .withTimeout(2.0) // Safety: Stop if it doesn't hit the limit in 2 seconds
        .withName("ExtenderHoming");
  }

  public Command followJoystick(DoubleSupplier speedSupplier) {
    return this.run(() -> {
      double speed = speedSupplier.getAsDouble();
      // Apply a small deadband so it doesn't hum when you aren't touching it
      if (Math.abs(speed) < 0.1 || m_isFaulted) {
        stopMotors();
      } else {
        double manualSpeed = speed * ExtenderConstants.kMotorSpeed;

        // Safety: Don't allow driving forward if already at/past max extension
        if (manualSpeed > 0 && m_LeaderEncoder.getPosition() >= Units.inchesToMeters(maxExtensionInches.get())) {
          manualSpeed = 0;
        }
        // Safety: Don't allow driving backward if already at/past stow
        if (manualSpeed < 0 && m_LeaderEncoder.getPosition() <= 0.0) {
          manualSpeed = 0;
        }

        m_LeaderMotor.set(manualSpeed);
        m_FollowMotor.set(manualSpeed);
      }
    }).withName("ExtenderManual");
  }

  public Command stopCommand() {
    return this.runOnce(this::stopMotors).withName("ExtenderStop");
  }

  public Command clearFaultsCommand() {
    return this.runOnce(() -> {
      this.m_isFaulted = false;
      this.stopMotors();
    }).withName("ExtenderClearFaults");
  }

  public SparkMax getLeader() {
    return m_LeaderMotor;
  }

  public SparkMax getFollower() {
    return m_FollowMotor;
  }
}

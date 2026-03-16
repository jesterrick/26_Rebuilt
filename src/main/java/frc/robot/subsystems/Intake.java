// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.configs.IntakeConfigs;
import frc.robot.constants.CanIdConstants;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.IntakeConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.Telemetry;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final SparkMax m_IntakeMotor;
  private final LoggedTunableNumber intakeSpeed = new LoggedTunableNumber("Intake/Speed", IntakeConstants.kMotorSpeed);

  public Intake() {
    this.m_IntakeMotor = new SparkMax(CanIdConstants.kIntakeMotor, MotorType.kBrushless);

    SparkMaxConfig intakeMotorConfig = new SparkMaxConfig().apply(IntakeConfigs.config);
    this.m_IntakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Telemetry.putBoolean("Intake/Jamming", isJamming());
  }

  public boolean isJamming() {
    // If current is high (> 30A) and velocity is low (< 0.1) while we are trying to run
    return m_IntakeMotor.getOutputCurrent() > GlobalConstants.kMediumCurrentLimit && Math.abs(m_IntakeMotor.getEncoder().getVelocity()) < 0.1;
  }

  private void runMotor(double speed) {
    this.m_IntakeMotor.set(speed);
  }

  private void stopMotor() {
    this.m_IntakeMotor.stopMotor();
  }

  public Command receive() {
    return this.run(() -> this.runMotor(intakeSpeed.get()))
        //.withTimeout(10.0) // Safety timeout
        .withName("IntakeReceive")
        .finallyDo(this::stopMotor);
  }

  public Command stop() {
    return this.run(() -> this.stopMotor());
  }

  public SparkMax getMotor() {
    return this.m_IntakeMotor;
  }
}

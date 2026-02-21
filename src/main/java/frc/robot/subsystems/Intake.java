// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.configs.IntakeConfigs;
import frc.robot.constants.CanIdConstants;
import frc.robot.constants.IntakeConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final SparkMax m_IntakeMotor;

  public Intake() {
    this.m_IntakeMotor = new SparkMax(CanIdConstants.kIntakeMotor, MotorType.kBrushless);

    SparkMaxConfig frontMotorConfig = new SparkMaxConfig().apply(IntakeConfigs.config);
    frontMotorConfig.inverted(true);
    this.m_IntakeMotor.configure(frontMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void runMotor(double speed) {
    this.m_IntakeMotor.set(speed);
  }

  private void stopMotor() {
    this.m_IntakeMotor.stopMotor();
  }

  public Command receive() {
    return this.run(() -> this.runMotor(IntakeConstants.kMotorSpeed))
        .withName("FeederOn")
        .finallyDo(this::stopMotor);
  }

  public Command stop() {
    return this.run(() -> this.stopMotor());
  }

  public SparkMax getMotor() {
    return this.m_IntakeMotor;
  }
}

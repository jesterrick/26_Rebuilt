// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.configs.FeederConfigs;
import frc.robot.constants.CanIdConstants;
import frc.robot.constants.FeederConstants;
import frc.robot.constants.GlobalConstants;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.Telemetry;

import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  private final SparkMax m_FeederMotor;
  private final LoggedTunableNumber feederSpeed = new LoggedTunableNumber("Feeder/Speed", FeederConstants.kMotorSpeed);
  /** Creates a new Feeder. */
  public Feeder() {
    this.m_FeederMotor = new SparkMax(CanIdConstants.kFeederMotor, MotorType.kBrushless);

    SparkMaxConfig feederMotorConfig = new SparkMaxConfig().apply(FeederConfigs.config);
    this.m_FeederMotor.configure(feederMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Telemetry.putBoolean("Feeder/Jamming", isJamming());
  }

  public boolean isJamming() {
    // If current is high (> 30A) and velocity is low (< 0.1) while we are trying to run
    return m_FeederMotor.getOutputCurrent() > GlobalConstants.kMediumCurrentLimit && Math.abs(m_FeederMotor.getEncoder().getVelocity()) < 0.1;
  }

  private void runMotor(double speed) {
    this.m_FeederMotor.set(speed);
  }

  private void stopMotor() {
    this.m_FeederMotor.stopMotor();
  }

  public Command feedLauncher() {
    return this.run(() -> this.runMotor(feederSpeed.get()))
    .withTimeout(5.0) // Safety timeout
    .withName("FeederOn")
    .finallyDo(this::stopMotor);
  }

  public SparkMax getMotor(){
    return this.m_FeederMotor;
  }
}

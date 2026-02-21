// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.configs.FeederConfigs;
import frc.robot.constants.CanIdConstants;
import frc.robot.constants.FeederConstants;

import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  private final SparkMax m_FeederMotor;
  /** Creates a new Feeder. */
  public Feeder() {
    this.m_FeederMotor = new SparkMax(CanIdConstants.kFeederMotor, MotorType.kBrushless);

    SparkMaxConfig frontMotorConfig = new SparkMaxConfig().apply(FeederConfigs.config);
    frontMotorConfig.inverted(true);
    this.m_FeederMotor.configure(frontMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void runMotor(double speed) {
    this.m_FeederMotor.set(speed);
  }

  private void stopMotor() {
    this.m_FeederMotor.stopMotor();
  }

  public Command feedLauncher() {
    return this.run(() -> this.runMotor(FeederConstants.kMotorSpeed))
    .withName("FeederOn")
    .finallyDo(this::stopMotor);
  }

  public SparkMax getMotor(){
    return this.m_FeederMotor;
  }
}

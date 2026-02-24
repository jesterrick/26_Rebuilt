package frc.robot.configs;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.constants.DriveConstants;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.MaxSwerveConstants;

public final class DriveConfigs {
  public static final class MAXSwerveModule {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor = MaxSwerveConstants.kWheelDiameterMeters * Math.PI
          / MaxSwerveConstants.kDrivingMotorReduction;
      double turningFactor = 2 * Math.PI;
      double drivingVelocityFeedForward = 1 / MaxSwerveConstants.kDriveWheelFreeSpeedRps;

      drivingConfig
          .idleMode(IdleMode.kBrake)
          .voltageCompensation(GlobalConstants.kMedVoltageCompensation)
          .smartCurrentLimit(GlobalConstants.kHighCurrentLimit);
      drivingConfig.encoder
          .positionConversionFactor(drivingFactor) // meters
          .velocityConversionFactor(drivingFactor / 60.0); // meters per second
      drivingConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(DriveConstants.kDriveP, DriveConstants.kDriveI, DriveConstants.kDriveD)
          .outputRange(-1, 1);

      drivingConfig.closedLoop.feedForward
          .kV(drivingVelocityFeedForward)
          .kS(DriveConstants.kDriveS);

      turningConfig
          .idleMode(IdleMode.kBrake)
          .voltageCompensation(GlobalConstants.kLowVoltageCompensation)
          .smartCurrentLimit(GlobalConstants.kLowCurrentLimit);
      turningConfig.absoluteEncoder
          // Invert the turning encoder, since the output shaft rotates in the opposite
          // direction of the steering motor in the MAXSwerve Module.
          .inverted(true)
          .positionConversionFactor(turningFactor) // radians
          .velocityConversionFactor(turningFactor / 60.0); // radians per second
      turningConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD)
          .outputRange(-1, 1)
          // Enable PID wrap around for the turning motor. This will allow the PID
          // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
          // to 10 degrees will go through 0 rather than the other direction which is a
          // longer route.
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, turningFactor);
    }
  }
}

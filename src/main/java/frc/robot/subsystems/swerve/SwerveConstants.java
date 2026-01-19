// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import frc.robot.util.PhoenixUtil.ClosedLoopControlType;

public final class SwerveConstants {
  private SwerveConstants() {}

  public static final LinearVelocity maxSpeed = MetersPerSecond.of(4.8);
  public static final double odometryFrequencyHz = 100.0; // Hz
  public static final Distance trackWidth = Inches.of(22.5);
  public static final Distance wheelBase = Inches.of(22.5);
  public static final Distance driveBaseRadius =
      Inches.of(Math.hypot(trackWidth.div(2.0).in(Inches), wheelBase.div(2.0).in(Inches)));
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth.div(2.0), wheelBase.div(2.0)),
        new Translation2d(trackWidth.div(2.0), wheelBase.div(2.0).unaryMinus()),
        new Translation2d(trackWidth.div(2.0).unaryMinus(), wheelBase.div(2.0)),
        new Translation2d(trackWidth.div(2.0).unaryMinus(), wheelBase.div(2.0).unaryMinus())
      };

  public static final boolean enableBeeps = true;

  // Controller Deadband for swerve
  public static final double controllerDeadband = 0.05;

  // Angle Controller used in commands
  public static final double angleControllerKp = 4.0;
  public static final double angleControllerKd = 0.4;
  public static final double allowedMaxAngularVelocity = 8.0;
  public static final double allowedMaxAngularAcceleration = 20.0;

  // Used for characterization
  public static final double FF_START_DELAY = 2.0; // Secs
  public static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  public static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  public static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
  public static final double SLOW_SPEED_PERCENTAGE =
      0.75; // Percentage of full speed when in slow mode

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(0.0);

  // Device CAN IDs
  public static final int gyroCanId = 11;

  public static final int frontLeftDriveCanId = 12;
  public static final int backLeftDriveCanId = 11;
  public static final int frontRightDriveCanId = 13;
  public static final int backRightDriveCanId = 14;

  public static final int frontLeftAzimuthCanId = 12;
  public static final int backLeftAzimuthCanId = 11;
  public static final int frontRightAzimuthCanId = 13;
  public static final int backRightAzimuthCanId = 14;

  // Drive motor configuration
  public static final Current driveStatorCurrentLimit = Amps.of(80);
  public static final Current driveMotorSupplyLimitHigh = Amps.of(80);
  public static final Current driveMotorSupplyLimitLow = Amps.of(40);
  public static final Time driveSupplyCurrentLowerTime = Seconds.of(1);
  public static final Distance wheelDiameter = Inches.of(3.0); // 2.84
  public static final Distance wheelRadius = wheelDiameter.div(2);
  public static final double driveMotorReduction = (45.0 * 28.0) / (20.0 * 15.0);
  public static final DCMotor driveGearbox = DCMotor.getKrakenX60Foc(1);
  public static final boolean driveMotorInverted = false;
  public static final ClosedLoopControlType driveMotorClosedLoopOutput =
      ClosedLoopControlType.Voltage;

  // Drive PID configuration
  public static final double driveKp = 0.05;
  public static final double driveKi = 0.0;
  public static final double driveKd = 0.0;

  public static final double driveKs = 0.15;
  public static final double driveKv = 0.5;
  public static final double driveKa = 0.0;

  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  public static final TalonFXConfiguration driveConfiguration =
      new TalonFXConfiguration()
          .withAudio(
              new AudioConfigs()
                  .withAllowMusicDurDisable(enableBeeps)
                  .withBeepOnBoot(enableBeeps)
                  .withBeepOnConfig(enableBeeps))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(driveStatorCurrentLimit)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(driveMotorSupplyLimitHigh)
                  .withSupplyCurrentLowerLimit(driveMotorSupplyLimitLow)
                  .withSupplyCurrentLowerTime(driveSupplyCurrentLowerTime)
                  .withSupplyCurrentLimitEnable(true))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  .withSensorToMechanismRatio(driveMotorReduction))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withSlot0(
              new Slot0Configs()
                  .withKP(driveKp)
                  .withKI(driveKi)
                  .withKD(driveKd)
                  .withKG(0)
                  .withKV(driveKv)
                  .withKS(driveKs)
                  .withKA(driveKa))
          .withTorqueCurrent(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(driveStatorCurrentLimit)
                  .withPeakReverseTorqueCurrent(driveStatorCurrentLimit.unaryMinus()));

  // Azimuth motor configuration
  public static final Current azimuthStatorCurrentLimit = Amps.of(25);
  public static final Current azimuthMotorSupplyLimitHigh = Amps.of(20);
  public static final Current azimuthMotorSupplyLimitLow = Amps.of(10);
  public static final Time azimuthSupplyCurrentLowerTime = Seconds.of(1);
  public static final double azimuthMotorReduction = (32 * 20 * 63) / (8 * 14 * 18);
  public static final DCMotor azimuthGearbox = DCMotor.getKrakenX44(1);
  public static final boolean azimuthMotorInverted = true;
  public static final ClosedLoopControlType azimuthMotorClosedLoopOutput =
      ClosedLoopControlType.Voltage;

  // Azimuth encoder configuration
  public static final boolean azimuthEncoderInverted = false;
  public static final double azimuthEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double azimuthEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Azimuth PID configuration
  public static final double azimuthKp = 1.0;
  public static final double azimuthKi = 0.0;
  public static final double azimuthKd = 0.0;

  public static final double azimuthKs = 0.0;
  public static final double azimuthKv = 0.0;
  public static final double azimuthKa = 0.0;

  public static final double azimuthSimP = 8.0;
  public static final double azimuthSimD = 0.0;
  public static final double azimuthSimKs = 0.0;
  public static final double azimuthSimKv = 0.0;

  public static final double azimuthPIDMinInput = 0; // Radians
  public static final double azimuthPIDMaxInput = 2 * Math.PI; // Radians

  public static final TalonFXConfiguration azimuthConfiguration =
      new TalonFXConfiguration()
          .withAudio(
              new AudioConfigs()
                  .withAllowMusicDurDisable(enableBeeps)
                  .withBeepOnBoot(enableBeeps)
                  .withBeepOnConfig(enableBeeps))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(azimuthStatorCurrentLimit)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(azimuthMotorSupplyLimitHigh)
                  .withSupplyCurrentLowerLimit(azimuthMotorSupplyLimitLow)
                  .withSupplyCurrentLowerTime(1)
                  .withSupplyCurrentLimitEnable(true))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                  .withFeedbackRemoteSensorID(0)
                  .withSensorToMechanismRatio(1)
                  .withRotorToSensorRatio(azimuthMotorReduction))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withSlot0(
              new Slot0Configs()
                  .withKP(azimuthKp)
                  .withKI(azimuthKi)
                  .withKD(azimuthKd)
                  .withKG(0)
                  .withKV(azimuthKv)
                  .withKS(azimuthKs)
                  .withKA(azimuthKa))
          .withTorqueCurrent(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(azimuthStatorCurrentLimit)
                  .withPeakReverseTorqueCurrent(azimuthStatorCurrentLimit.unaryMinus()));

  // PathPlanner configuration
  public static final Mass robotMassKg = Pounds.of(115);
  public static final MomentOfInertia robotMOI = KilogramSquareMeters.of(10);
  public static final double wheelCOF = .5;
  public static final PathConstraints pathConstraints =
      new PathConstraints(
          MetersPerSecond.of(5),
          MetersPerSecondPerSecond.of(2),
          RadiansPerSecond.of(3),
          RadiansPerSecondPerSecond.of(2));
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadius,
              maxSpeed,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveStatorCurrentLimit,
              1),
          moduleTranslations);
}

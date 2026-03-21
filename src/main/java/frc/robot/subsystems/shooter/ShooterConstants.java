// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;

/** Add your docs here. */
public class ShooterConstants {

  // Shooter Offset
  public static final Translation2d shooterTranslation =
      new Translation2d(Meters.of(-0.090488), Meters.of(0));
  public static final Transform2d ShooterTransorm =
      new Transform2d(Meters.of(-0.090488), Meters.of(0), new Rotation2d());

  public static boolean enableBeeps = true;

  public static final int shooterMotor1CanID = 50;
  public static final int shooterMotor2CanID = 51;
  public static final int turretMotorCanID = 52;
  public static final int hoodMotorCanID = 53;
  public static final int TurretEncoderPin = 0;

  // Shooter Motor 1
  public static final Current shooterMotor1StatorCurrentLimit = Amps.of(120);
  public static final Current shooterMotor1MotorSupplyLimitHigh = Amps.of(80);
  public static final Current shooterMotor1MotorSupplyLimitLow = Amps.of(40);
  public static final Time shooterMotor1SupplyCurrentLowerTime = Seconds.of(1);

  public static final double shooterMotor1MotorReduction = 1.0;

  public static final InvertedValue shooterMotor1Inverted = InvertedValue.Clockwise_Positive;
  public static final NeutralModeValue shooterMotor1NeutralMode = NeutralModeValue.Coast;

  public static final double shooterKp = 0.5;
  public static final double shooterKi = 0.0;
  public static final double shooterKd = 0.0;
  public static final double shooterKv = 0.126;
  public static final double shooterKs = 0.2;
  public static final double shooterKa = 0.0;

  // Shooter Motor 2
  public static final Current shooterMotor2StatorCurrentLimit = Amps.of(120);
  public static final Current shooterMotor2MotorSupplyLimitHigh = Amps.of(80);
  public static final Current shooterMotor2MotorSupplyLimitLow = Amps.of(40);
  public static final Time shooterMotor2SupplyCurrentLowerTime = Seconds.of(1);

  public static final double shooterMotor2MotorReduction = 1.0;

  public static final InvertedValue shooterMotor2Inverted = InvertedValue.CounterClockwise_Positive;
  public static final NeutralModeValue shooterMotor2NeutralMode = NeutralModeValue.Coast;

  // Turret Motor
  public static final Current turretMotorStatorCurrentLimit = Amps.of(100);
  public static final Current turretMotorMotorSupplyLimitHigh = Amps.of(40);
  public static final Current turretMotorMotorSupplyLimitLow = Amps.of(30);
  public static final Time turretMotorSupplyCurrentLowerTime = Seconds.of(1);

  public static final double turretMotorMotorReduction = (110.0 / 16.0) * (42.0 / 10.0);

  public static final InvertedValue turretMotorInverted = InvertedValue.Clockwise_Positive;
  public static final NeutralModeValue turretMotorNeutralMode = NeutralModeValue.Brake;

  public static final double turretMotorKp = 100;
  public static final double turretMotorKi = 0.0;
  public static final double turretMotorKd = 1;
  public static final double turretMotorKv = 3.125;
  public static final double turretMotorKs = 0.4;
  public static final double turretMotorKa = 0.0;

  public static final AngularVelocity turretcruiseVelocity = RotationsPerSecond.of(3.5);
  public static final AngularAcceleration turretAcceleration = RotationsPerSecondPerSecond.of(50);

  // Hood Motor
  public static final int hoodStatorCurrentLimit = 20;
  public static final double hoodMotorReduction = (18.0 * 18.0 * 15.0) / (40.0 * 43.0 * 298.0);
  public static final boolean hoodInverted = true;
  public static final IdleMode hoodNeutralMode = IdleMode.kBrake;

  public static final double hoodKp = 100.0;
  public static final double hoodKi = 0.0;
  public static final double hoodKd = 1.0;
  public static final double hoodKv = 0.0;
  public static final double hoodKs = 0.1;
  public static final double hoodKa = 0.0;

  // Shot calculator parameters
  public static final double phaseDelay = 0.03;

  // Launching Maps
  public static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  public static final InterpolatingDoubleTreeMap flywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

  // Passing Maps
  public static final InterpolatingTreeMap<Double, Rotation2d> passingHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  public static final InterpolatingDoubleTreeMap passingFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap passingTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    hoodAngleMap.put(1.12, new Rotation2d(Degrees.of(2)));
    flywheelSpeedMap.put(1.12, 31.0);
    timeOfFlightMap.put(1.12, 2.1825);

    hoodAngleMap.put(5.56, new Rotation2d(Degrees.of(30)));
    flywheelSpeedMap.put(5.56, 42.0);
    timeOfFlightMap.put(5.56, 2.1825); // TODO: get time
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;

/** Add your docs here. */
public class ShooterConstants {

  public static boolean enableBeeps = true;

  public static final int shooterMotor1CanID = 61;
  public static final int shooterMotor2CanID = 62;
  public static final int turretMotorCanID = 63;
  public static final int hoodMotorCanID = 64;

  // Shooter Motor 1
  public static final Current shooterMotor1StatorCurrentLimit = Amps.of(10);
  public static final Current shooterMotor1MotorSupplyLimitHigh = Amps.of(10);
  public static final Current shooterMotor1MotorSupplyLimitLow = Amps.of(10);
  public static final Time shooterMotor1SupplyCurrentLowerTime = Seconds.of(1);

  public static final double shooterMotor1MotorReduction = 1.0;

  public static final InvertedValue shooterMotor1Inverted = InvertedValue.Clockwise_Positive;
  public static final NeutralModeValue shooterMotor1NeutralMode = NeutralModeValue.Coast;

  public static final double shooterMotor1Kp = 0.0;
  public static final double shooterMotor1Ki = 0.0;
  public static final double shooterMotor1Kd = 0.0;
  public static final double shooterMotor1Kv = 0.0;
  public static final double shooterMotor1Ks = 0.0;
  public static final double shooterMotor1Ka = 0.0;

  // Shooter Motor 2
  public static final Current shooterMotor2StatorCurrentLimit = Amps.of(10);
  public static final Current shooterMotor2MotorSupplyLimitHigh = Amps.of(10);
  public static final Current shooterMotor2MotorSupplyLimitLow = Amps.of(10);
  public static final Time shooterMotor2SupplyCurrentLowerTime = Seconds.of(1);

  public static final double shooterMotor2MotorReduction = 1.0;

  public static final InvertedValue shooterMotor2Inverted = InvertedValue.Clockwise_Positive;
  public static final NeutralModeValue shooterMotor2NeutralMode = NeutralModeValue.Coast;

  public static final double shooterMotor2Kp = 0.0;
  public static final double shooterMotor2Ki = 0.0;
  public static final double shooterMotor2Kd = 0.0;
  public static final double shooterMotor2Kv = 0.0;
  public static final double shooterMotor2Ks = 0.0;
  public static final double shooterMotor2Ka = 0.0;

  // Turret Motor
  public static final Current turretMotorStatorCurrentLimit = Amps.of(10);
  public static final Current turretMotorMotorSupplyLimitHigh = Amps.of(10);
  public static final Current turretMotorMotorSupplyLimitLow = Amps.of(10);
  public static final Time turretMotorSupplyCurrentLowerTime = Seconds.of(1);

  public static final double turretMotorMotorReduction = 3.0;

  public static final InvertedValue turretMotorInverted = InvertedValue.Clockwise_Positive;
  public static final NeutralModeValue turretMotorNeutralMode = NeutralModeValue.Coast;

  public static final double turretMotorKp = 0.0;
  public static final double turretMotorKi = 0.0;
  public static final double turretMotorKd = 0.0;
  public static final double turretMotorKv = 0.0;
  public static final double turretMotorKs = 0.0;
  public static final double turretMotorKa = 0.0;

  // Hood Motor
  public static final Current hoodStatorCurrentLimit = Amps.of(10);
  public static final Current hoodMotorSupplyLimitHigh = Amps.of(10);
  public static final Current hoodMotorSupplyLimitLow = Amps.of(10);
  public static final Time hoodSupplyCurrentLowerTime = Seconds.of(1);

  public static final double hoodMotorReduction = 3.0;

  public static final InvertedValue hoodInverted = InvertedValue.Clockwise_Positive;
  public static final NeutralModeValue hoodNeutralMode = NeutralModeValue.Coast;

  public static final double hoodKp = 0.0;
  public static final double hoodKi = 0.0;
  public static final double hoodKd = 0.0;
  public static final double hoodKv = 0.0;
  public static final double hoodKs = 0.0;
  public static final double hoodKa = 0.0;
}

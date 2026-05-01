// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.dumper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;

/** Add your docs here. */
public class DumperConstants {

  public static boolean enableBeeps = true;

  // Motor 1dumperMotor1CanID
  public static final int dumperMotor1CanID = 41;
  public static final int dumperMotor2CanID = 42;
  public static final int dumperMotor3CanID = 43;
  public static final int dumperMotor4CanID = 44;

  public static final Current dumperMotor1StatorCurrentLimit = Amps.of(10);
  public static final Current dumperMotor1MotorSupplyLimitHigh = Amps.of(10);
  public static final Current dumperMotor1MotorSupplyLimitLow = Amps.of(10);
  public static final Time dumperMotor1SupplyCurrentLowerTime = Seconds.of(1);

  public static final double dumperMotor1MotorReduction = 3.0;

  public static final InvertedValue dumperMotor1Inverted = InvertedValue.Clockwise_Positive;
  public static final NeutralModeValue dumperMotor1NeutralMode = NeutralModeValue.Coast;

  public static final double dumperMotor1Kp = 0.0;
  public static final double dumperMotor1Ki = 0.0;
  public static final double dumperMotor1Kd = 0.0;
  public static final double dumperMotor1Kv = 0.0;
  public static final double dumperMotor1Ks = 0.0;
  public static final double dumperMotor1Ka = 0.0;

  // Motor 2
  public static final Current dumperMotor2StatorCurrentLimit = Amps.of(10);
  public static final Current dumperMotor2MotorSupplyLimitHigh = Amps.of(10);
  public static final Current dumperMotor2MotorSupplyLimitLow = Amps.of(10);
  public static final Time dumperMotor2SupplyCurrentLowerTime = Seconds.of(1);

  public static final double dumperMotor2MotorReduction = 3.0;

  public static final InvertedValue dumperMotor2Inverted = InvertedValue.Clockwise_Positive;
  public static final NeutralModeValue dumperMotor2NeutralMode = NeutralModeValue.Coast;

  public static final double dumperMotor2Kp = 0.0;
  public static final double dumperMotor2Ki = 0.0;
  public static final double dumperMotor2Kd = 0.0;
  public static final double dumperMotor2Kv = 0.0;
  public static final double dumperMotor2Ks = 0.0;
  public static final double dumperMotor2Ka = 0.0;

  // Motor 3
  public static final Current dumperMotor3StatorCurrentLimit = Amps.of(10);
  public static final Current dumperMotor3MotorSupplyLimitHigh = Amps.of(10);
  public static final Current dumperMotor3MotorSupplyLimitLow = Amps.of(10);
  public static final Time dumperMotor3SupplyCurrentLowerTime = Seconds.of(1);

  public static final double dumperMotor3MotorReduction = 3.0;

  public static final InvertedValue dumperMotor3Inverted = InvertedValue.Clockwise_Positive;
  public static final NeutralModeValue dumperMotor3NeutralMode = NeutralModeValue.Coast;

  public static final double dumperMotor3Kp = 0.0;
  public static final double dumperMotor3Ki = 0.0;
  public static final double dumperMotor3Kd = 0.0;
  public static final double dumperMotor3Kv = 0.0;
  public static final double dumperMotor3Ks = 0.0;
  public static final double dumperMotor3Ka = 0.0;

  // Motor 4
  public static final Current dumperMotor4StatorCurrentLimit = Amps.of(10);
  public static final Current dumperMotor4MotorSupplyLimitHigh = Amps.of(10);
  public static final Current dumperMotor4MotorSupplyLimitLow = Amps.of(10);
  public static final Time dumperMotor4SupplyCurrentLowerTime = Seconds.of(1);

  public static final double dumperMotor4MotorReduction = 3.0;

  public static final InvertedValue dumperMotor4Inverted = InvertedValue.Clockwise_Positive;
  public static final NeutralModeValue dumperMotor4NeutralMode = NeutralModeValue.Coast;

  public static final double dumperMotor4Kp = 0.0;
  public static final double dumperMotor4Ki = 0.0;
  public static final double dumperMotor4Kd = 0.0;
  public static final double dumperMotor4Kv = 0.0;
  public static final double dumperMotor4Ks = 0.0;
  public static final double dumperMotor4Ka = 0.0;
}

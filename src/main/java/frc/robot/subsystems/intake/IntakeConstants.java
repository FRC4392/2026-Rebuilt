// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.InchesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class IntakeConstants {

  public static final boolean enableBeeps = true;

  // CAN IDs
  public static final int extensionCanID = 21;
  public static final int leftRollerCanID = 22;
  public static final int rightRollerCanID = 23;

  // Extension Parameters
  public static final Current extenstionStatorCurrentLimit = Amps.of(30);
  public static final Current extensionMotorSupplyLimitHigh = Amps.of(30);
  public static final Current extensionMotorSupplyLimitLow = Amps.of(20);
  public static final Time extensionSupplyCurrentLowerTime = Seconds.of(1);

  public static final double extensionMotorReduction = 56.0 / 10.0;
  public static final Distance extensionDriveDiameter = Inches.of(16.0 / 10.0);

  public static final InvertedValue extensionInverted = InvertedValue.Clockwise_Positive;
  public static final NeutralModeValue extensionNeutralMode = NeutralModeValue.Brake;

  public static final double extensionKp = 0.0;
  public static final double extensionKi = 0.0;
  public static final double extensionKd = 0.0;
  public static final double extensionKv = 0.0;
  public static final double extensionKs = 0.0;
  public static final double extensionKa = 0.0;

  public static final Voltage extensionTestVoltage = Volts.of(3);

  public static final LinearVelocity extensionMaxSpeed = InchesPerSecond.of(1);
  public static final LinearAcceleration extensionMaxAcceleration = InchesPerSecondPerSecond.of(1);

  public static final AngularVelocity extensionMotorMaxSpeed =
      RadiansPerSecond.of(
          extensionMaxSpeed.in(MetersPerSecond) / (extensionDriveDiameter.in(Meters) / 2.0));
  public static final AngularAcceleration extensionMotorMaxAcceleration =
      RadiansPerSecondPerSecond.of(
          extensionMaxAcceleration.in(MetersPerSecondPerSecond)
              / (extensionDriveDiameter.in(Meters) / 2.0));

  // Roller Parameters
  public static final Current rollerStatorCurrentLimit = Amps.of(70);
  public static final Current rollerMotorSupplyLimitHigh = Amps.of(60);
  public static final Current rollerMotorSupplyLimitLow = Amps.of(30);
  public static final Time rollerSupplyCurrentLowerTime = Seconds.of(1);

  public static final double rollerMotorReduction = 35.0 / 11.0;
  public static final Distance rollerDriveDiameter = Inches.of(4.0);

  public static final InvertedValue leftRollerInverted = InvertedValue.CounterClockwise_Positive;
  public static final InvertedValue rightRollerInverted = InvertedValue.Clockwise_Positive;
  public static final NeutralModeValue rollerNeutralMode = NeutralModeValue.Brake;

  public static final Voltage intakeVoltage = Volts.of(10);
  public static final Voltage outtakeVoltage = Volts.of(-10);
}

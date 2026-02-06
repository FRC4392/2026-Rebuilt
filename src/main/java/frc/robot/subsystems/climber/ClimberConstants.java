// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;

/** Add your docs here. */
public class ClimberConstants {

  public static boolean enableBeeps = true;

  public static final int climberMotorCanID = 41;

  public static final Current climberStatorCurrentLimit = Amps.of(10);
  public static final Current climberMotorSupplyLimitHigh = Amps.of(10);
  public static final Current climberMotorSupplyLimitLow = Amps.of(10);
  public static final Time climberSupplyCurrentLowerTime = Seconds.of(1);

  public static final double climberMotorReduction = 3.0;

  public static final InvertedValue climberInverted = InvertedValue.Clockwise_Positive;
  public static final NeutralModeValue climberNeutralMode = NeutralModeValue.Coast;

  public static final double climberKp = 0.0;
  public static final double climberKi = 0.0;
  public static final double climberKd = 0.0;
  public static final double climberKv = 0.0;
  public static final double climberKs = 0.0;
  public static final double climberKa = 0.0;
}

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

  public static final int dumperMotorCanID = 41;

  public static final Current dumperStatorCurrentLimit = Amps.of(10);
  public static final Current dumperMotorSupplyLimitHigh = Amps.of(10);
  public static final Current dumperMotorSupplyLimitLow = Amps.of(10);
  public static final Time dumperSupplyCurrentLowerTime = Seconds.of(1);

  public static final double dumperMotorReduction = 3.0;

  public static final InvertedValue dumperInverted = InvertedValue.Clockwise_Positive;
  public static final NeutralModeValue dumperNeutralMode = NeutralModeValue.Coast;

  public static final double dumperKp = 0.0;
  public static final double dumperKi = 0.0;
  public static final double dumperKd = 0.0;
  public static final double dumperKv = 0.0;
  public static final double dumperKs = 0.0;
  public static final double dumperKa = 0.0;
}

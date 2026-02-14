// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;

/** Add your docs here. */
public class HopperConstants {

  public static boolean enableBeeps = true;

  public static final int hopperMotorCanID = 41;

  public static final Current hopperStatorCurrentLimit = Amps.of(10);
  public static final Current hopperMotorSupplyLimitHigh = Amps.of(10);
  public static final Current hopperMotorSupplyLimitLow = Amps.of(10);
  public static final Time hopperSupplyCurrentLowerTime = Seconds.of(1);

  public static final double hopperMotorReduction = 3.0;

  public static final InvertedValue hopperInverted = InvertedValue.Clockwise_Positive;
  public static final NeutralModeValue hopperNeutralMode = NeutralModeValue.Coast;

  public static final double hopperKp = 0.0;
  public static final double hopperKi = 0.0;
  public static final double hopperKd = 0.0;
  public static final double hopperKv = 0.0;
  public static final double hopperKs = 0.0;
  public static final double hopperKa = 0.0;
}

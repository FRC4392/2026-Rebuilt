// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;

/** Add your docs here. */
public class HopperConstants {

  public static boolean enableBeeps = true;

  public static final int hopperMotorCanID = 41;
  public static final int topRollerCanID = 42;

  public static final Current hopperStatorCurrentLimit = Amps.of(100);
  public static final Current hopperMotorSupplyLimitHigh = Amps.of(80);
  public static final Current hopperMotorSupplyLimitLow = Amps.of(20);
  public static final Time hopperSupplyCurrentLowerTime = Seconds.of(1);

  public static final double hopperMotorReduction = 3.0;

  public static final InvertedValue hopperInverted = InvertedValue.CounterClockwise_Positive;
  public static final NeutralModeValue hopperNeutralMode = NeutralModeValue.Coast;

  public static final boolean topRollerInverted = true;
  public static final int topRollerCurrentLimit = 80;
  public static final IdleMode topRollerIdleMode = IdleMode.kCoast;
  public static final double topRollerRatio = (12.0 / 26.0);

  public static final double hopperKp = 0.0;
  public static final double hopperKi = 0.0;
  public static final double hopperKd = 0.0;
  public static final double hopperKv = 0.0;
  public static final double hopperKs = 0.0;
  public static final double hopperKa = 0.0;
}

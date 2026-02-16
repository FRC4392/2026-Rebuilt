// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;

/** Add your docs here. */
public class IntakeConstants {

  public static final boolean enableBeeps = true;

  public static final int intakeMotorCanID = 21;

  public static final Current intakeStatorCurrentLimit = Amps.of(10);
  public static final Current intakeMotorSupplyLimitHigh = Amps.of(10);
  public static final Current intakeMotorSupplyLimitLow = Amps.of(10);
  public static final Time intakeSupplyCurrentLowerTime = Seconds.of(1);

  public static final double intakeMotorReduction = 3.0;

  public static final InvertedValue intakeInverted = InvertedValue.Clockwise_Positive;
  public static final NeutralModeValue intakeNeutralMode = NeutralModeValue.Coast;

  public static final double intakeKp = 0.0;
  public static final double intakeKi = 0.0;
  public static final double intakeKd = 0.0;
  public static final double intakeKv = 0.0;
  public static final double intakeKs = 0.0;
  public static final double intakeKa = 0.0;
}

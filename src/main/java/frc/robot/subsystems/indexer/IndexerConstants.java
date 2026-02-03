// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;

/** Add your docs here. */
public class IndexerConstants {

    public static boolean enableBeeps = true;

    public static final int indexerMotorCanID = 31;

    public static final Current indexerStatorCurrentLimit = Amps.of(10);
    public static final Current indexerMotorSupplyLimitHigh = Amps.of(10);
    public static final Current indexerMotorSupplyLimitLow = Amps.of(10);
    public static final Time indexerSupplyCurrentLowerTime = Seconds.of(1);
    
    public static final double indexerMotorReduction = 3.0;

    public static final InvertedValue indexerInverted = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue indexerNeutralMode = NeutralModeValue.Coast;

    public static final double indexerKp = 0.0;
    public static final double indexerKi = 0.0;
    public static final double indexerKd = 0.0;
    public static final double indexerKv = 0.0;
    public static final double indexerKs = 0.0;
    public static final double indexerKa = 0.0;
}

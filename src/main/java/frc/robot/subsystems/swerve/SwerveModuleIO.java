// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Defines IO interface to a swerve module */
public interface SwerveModuleIO {
  /** Defines the inputs coming from a swerve module */
  @AutoLog
  public static class SwerveModuleIOInputs {
    public boolean driveConnected = false;
    public Angle drivePositionAngle = Radians.of(0.0);
    public AngularVelocity driveVelocity = RadiansPerSecond.of(0.0);
    public Voltage driveAppliedVolts = Volts.of(0.0);
    public Current driveCurrentAmps = Amps.of(0.0);
    public Temperature driveMotorTemp = Celsius.of(0.0);

    public boolean azimuthConnected = false;
    public Rotation2d azimuthPosition = new Rotation2d();
    public AngularVelocity azimuthVelocity = RadiansPerSecond.of(0.0);
    public Voltage azimuthAppliedVolts = Volts.of(0.0);
    public Current azimuthCurrent = Amps.of(0.0);
    public Temperature azimuthMotorTemp = Celsius.of(0.0);

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryAzimuthPositions = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(SwerveModuleIOInputs inputs) {}

  /** Run the drive motor at the specified open loop value. */
  // TODO: Convert to volts
  public default void setDriveOpenLoop(double output) {}

  /** Run the azimuth motor at the specified open loop value. */
  // TODO: Convert to volts
  public default void setAzimuthOpenLoop(double output) {}

  /** Run the drive motor at the specified velocity. */
  public default void setDriveVelocity(AngularVelocity velocityRadPerSec) {}

  /** Run the Azimuth motor to the specified rotation. */
  public default void setAzimuthPosition(Rotation2d rotation) {}
}

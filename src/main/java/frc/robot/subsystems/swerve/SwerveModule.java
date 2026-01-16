// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.swerve.SwerveConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

/** Represents a single swerve module */
public class SwerveModule {
  private final SwerveModuleIO io;
  private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
  private final int index;

  private final Alert driveDisconnectedAlert;
  private final Alert azimuthDisconnectedAlert;

  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  /**
   * Constructor of swerve module
   *
   * <p>Supply with Swerve IO instance and index number of module. Standard numbering is as follows:
   *
   * <ol>
   *   <li>front left module
   *   <li>front right module
   *   <li>back left module
   *   <li>back right module
   * </ol>
   *
   * @param io IO instance of the swerve module
   * @param index Number of the swerve module
   */
  public SwerveModule(SwerveModuleIO io, int index) {
    this.io = io;
    this.index = index;

    driveDisconnectedAlert =
        new Alert(
            "Disconnected drive motor on module " + Integer.toString(index) + ".",
            AlertType.kError);
    azimuthDisconnectedAlert =
        new Alert(
            "Disconnected azimuth motor on module " + Integer.toString(index) + ".",
            AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * wheelRadius.in(Meters);
      Rotation2d angle = inputs.odometryAzimuthPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    driveDisconnectedAlert.set(!inputs.driveConnected);
    azimuthDisconnectedAlert.set(!inputs.azimuthConnected);
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    // Optimize velocity setpoint
    state.optimize(getAngle());
    state.cosineScale(getAngle());

    // Apply setpoints
    io.setDrive(RadiansPerSecond.of(state.speedMetersPerSecond / wheelRadius.in(Meters)));
    io.setAzimuth(state.angle);
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(Voltage volts) {
    io.setDrive(volts);
    io.setAzimuth(new Rotation2d());
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setDrive(Volts.of(0));
    io.setAzimuth(Volts.of(0));
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.azimuthPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public Distance getDrivePosition() {
    return Meters.of(inputs.drivePositionAngle.in(Radians) * wheelRadius.in(Meters));
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public LinearVelocity getDriveVelocity() {
    return MetersPerSecond.of(inputs.driveVelocity.in(RadiansPerSecond) * wheelRadius.in(Meters));
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  // TODO: Units?
  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  // TODO: Units?
  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionAngle.in(Radians);
  }

  // TODO: Units?
  /** Returns the module velocity in rad/sec. */
  public double getFFCharacterizationVelocity() {
    return inputs.driveVelocity.in(RadiansPerSecond);
  }
}

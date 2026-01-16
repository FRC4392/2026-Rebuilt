// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.swerve.SwerveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Swerve drive simulation */
public class SwerveModuleIOSim implements SwerveModuleIO {
  private final DCMotorSim driveSim;
  private final DCMotorSim azimuthSim;

  private boolean driveClosedLoop = false;
  private boolean azimuthClosedLoop = false;
  private PIDController driveController = new PIDController(driveSimP, 0, driveSimD);
  private PIDController azimuthController = new PIDController(azimuthSimP, 0, azimuthSimD);
  private double driveFFVolts = 0.0;
  private Voltage driveAppliedVolts = Volts.of(0.0);
  private Voltage azimuthAppliedVolts = Volts.of(0.0);

  /** Constructor */
  public SwerveModuleIOSim() {
    // Create drive and azimuth sim models
    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveGearbox, 0.025, driveMotorReduction),
            driveGearbox);
    azimuthSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(azimuthGearbox, 0.004, azimuthMotorReduction),
            azimuthGearbox);

    // Enable wrapping for azimuth PID
    azimuthController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts =
          Volts.of(
              driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec()));
    } else {
      driveController.reset();
    }
    if (azimuthClosedLoop) {
      azimuthAppliedVolts =
          Volts.of(azimuthController.calculate(azimuthSim.getAngularPositionRad()));
    } else {
      azimuthController.reset();
    }

    // Update simulation state
    driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts.in(Volts), -12.0, 12.0));
    azimuthSim.setInputVoltage(MathUtil.clamp(azimuthAppliedVolts.in(Volts), -12.0, 12.0));
    driveSim.update(0.02);
    azimuthSim.update(0.02);

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionAngle = driveSim.getAngularPosition();
    inputs.driveVelocity = driveSim.getAngularVelocity();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Amps.of(Math.abs(driveSim.getCurrentDrawAmps()));

    // Update azimuth inputs
    inputs.azimuthConnected = true;
    inputs.azimuthPosition = new Rotation2d(azimuthSim.getAngularPositionRad());
    inputs.azimuthVelocity = azimuthSim.getAngularVelocity();
    inputs.azimuthAppliedVolts = azimuthAppliedVolts;
    inputs.azimuthCurrent = Amps.of(Math.abs(azimuthSim.getCurrentDrawAmps()));

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't
    // matter)
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionAngle.in(Radians)};
    inputs.odometryAzimuthPositions = new Rotation2d[] {inputs.azimuthPosition};
  }

  @Override
  public void setDrive(Voltage volts) {
    driveClosedLoop = false;
    driveAppliedVolts = volts;
  }

  @Override
  public void setAzimuth(Voltage volts) {
    azimuthClosedLoop = false;
    azimuthAppliedVolts = volts;
  }

  @Override
  public void setDrive(AngularVelocity velocity) {
    driveClosedLoop = true;
    driveFFVolts =
        driveSimKs * Math.signum(velocity.in(RadiansPerSecond))
            + driveSimKv * velocity.in(RadiansPerSecond);
    driveController.setSetpoint(velocity.in(RadiansPerSecond));
  }

  @Override
  public void setAzimuth(Rotation2d rotation) {
    azimuthClosedLoop = true;
    azimuthController.setSetpoint(rotation.getRadians());
  }
}

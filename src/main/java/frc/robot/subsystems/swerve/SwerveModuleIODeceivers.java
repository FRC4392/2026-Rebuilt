// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.*;
import static frc.robot.util.PhoenixUtil.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import java.util.Queue;

/** Represents a single swerve module IO */
public class SwerveModuleIODeceivers implements SwerveModuleIO {
  private final Rotation2d zeroRotation;

  // Hardware
  private final TalonFX azimuthMotor;
  private final TalonFX driveMotor;

  //TODO: do these need to exist?
  // Voltage control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  // Torque-current control requests
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  // Duty Cycle Control Requests
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);
  private final VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(0.0);

  //Position Control Requests
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);

  // Inputs from drive motor
  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;
  private final StatusSignal<Temperature> driveTemp;

  private final StatusSignal<Angle> azimuthPosition;
  private final StatusSignal<AngularVelocity> azimuthVelocity;
  private final StatusSignal<Voltage> azimuthAppliedVolts;
  private final StatusSignal<Current> azimuthCurrent;
  private final StatusSignal<Temperature> azimuthTemp;

  // Odometry queue
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer azimuthConnectedDebounce = new Debouncer(0.5);

  /**
   * Constructor for swerve module
   *
   * <p>Must supply a module number:
   *
   * <ol>
   *   <li>front left module
   *   <li>front right module
   *   <li>back left module
   *   <li>back right module
   * </ol>
   *
   * @param module Module number
   */
  public SwerveModuleIODeceivers(int module) {

    zeroRotation =
        switch (module) {
          case 0 -> frontLeftZeroRotation;
          case 1 -> frontRightZeroRotation;
          case 2 -> backLeftZeroRotation;
          case 3 -> backRightZeroRotation;
          default -> new Rotation2d();
        };

    azimuthMotor =
        new TalonFX(
            switch (module) {
              case 0 -> frontLeftAzimuthCanId;
              case 1 -> frontRightAzimuthCanId;
              case 2 -> backLeftAzimuthCanId;
              case 3 -> backRightAzimuthCanId;
              default -> 0;
            });

    driveMotor =
        new TalonFX(
            switch (module) {
              case 0 -> frontLeftDriveCanId;
              case 1 -> frontRightDriveCanId;
              case 2 -> backLeftDriveCanId;
              case 3 -> backRightDriveCanId;
              default -> 0;
            });

    tryUntilOk(5, () -> driveMotor.getConfigurator().apply(driveConfiguration, 0.25));
    tryUntilOk(5, () -> driveMotor.setPosition(0.0, 0.25));

    // Configure turn motor

    tryUntilOk(5, () -> azimuthMotor.getConfigurator().apply(azimuthConfiguration, 0.25));

    // Create drive status signals
    drivePosition = driveMotor.getPosition();
    driveVelocity = driveMotor.getVelocity();
    driveAppliedVolts = driveMotor.getMotorVoltage();
    driveCurrent = driveMotor.getStatorCurrent();
    driveTemp = driveMotor.getDeviceTemp();

    azimuthPosition = azimuthMotor.getPosition();
    azimuthVelocity = azimuthMotor.getVelocity();
    azimuthAppliedVolts = azimuthMotor.getMotorVoltage();
    azimuthCurrent = azimuthMotor.getStatorCurrent();
    azimuthTemp = azimuthMotor.getDeviceTemp();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(odometryFrequencyHz, drivePosition, azimuthPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        azimuthVelocity,
        azimuthAppliedVolts,
        azimuthCurrent);
    ParentDevice.optimizeBusUtilizationForAll(driveMotor, azimuthMotor);

    // Create odometry queues
    timestampQueue = SwerveOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SwerveOdometryThread.getInstance().registerSignal(driveMotor.getPosition());
    turnPositionQueue =
        SwerveOdometryThread.getInstance().registerSignal(azimuthMotor.getPosition());
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    // Update drive inputs
    var driveStatus =
        BaseStatusSignal.refreshAll(
            drivePosition, driveVelocity, driveAppliedVolts, driveCurrent, driveTemp);

    inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
    inputs.drivePositionAngle = drivePosition.getValue();
    inputs.driveVelocity = driveVelocity.getValue();
    inputs.driveAppliedVolts = driveAppliedVolts.getValue();
    inputs.driveCurrentAmps = driveCurrent.getValue();
    inputs.driveMotorTemp = driveTemp.getValue();

    // Update turn inputs

    var azimuthStatus =
        BaseStatusSignal.refreshAll(
            azimuthPosition, azimuthVelocity, azimuthAppliedVolts, azimuthCurrent, azimuthTemp);

    inputs.azimuthConnected = azimuthConnectedDebounce.calculate(azimuthStatus.isOK());
    inputs.azimuthPosition = new Rotation2d(azimuthPosition.getValue());
    inputs.azimuthVelocity = azimuthVelocity.getValue();
    inputs.azimuthAppliedVolts = azimuthAppliedVolts.getValue();
    inputs.azimuthCurrent = azimuthCurrent.getValue();
    inputs.azimuthMotorTemp = azimuthTemp.getValue();

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value))
            .toArray();
    inputs.odometryAzimuthPositions =
        turnPositionQueue.stream()
            .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  // TODO: Convert to volts
  @Override
  public void setAzimuthOpenLoop(double output) {
    azimuthMotor.setVoltage(output);
  }

  @Override
  public void setAzimuthPosition(Rotation2d rotation) {
    azimuthMotor.setControl(positionVoltageRequest.withPosition(rotation.getMeasure()));
  }

  @Override
  public void setDriveVelocity(AngularVelocity velocity) {
    // double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    driveMotor.setControl(
        switch (driveMotorClosedLoopOutput) {
          case Voltage -> velocityVoltageRequest.withVelocity(velocity);
          case TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(velocity);
          case DutyCyle -> velocityDutyCycle.withVelocity(velocity);
        });
  }

  // TODO: Convert to volts
  @Override
  public void setDriveOpenLoop(double output) {
    driveMotor.setControl(
        switch (driveMotorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
          case DutyCyle -> dutyCycleRequest.withOutput(output);
        });
  }
}

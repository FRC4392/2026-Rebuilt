// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.swerve.SwerveConstants.*;

import com.studica.frc.Navx;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Queue;

/** Gyro implementation using a pigeon2 */
public class GyroIONavx3 implements GyroIO {
  private final Navx gyro = new Navx(gyroCanId);
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  /** Constructor */
  public GyroIONavx3() {
    gyro.resetYaw();
    gyro.setODRHz((int) odometryFrequencyHz);
    yawTimestampQueue = SwerveOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = SwerveOdometryThread.getInstance().makeTimestampQueue();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.isConnected = gyro.getYaw() != Degrees.of(360);
    inputs.yawPosition = new Rotation2d(gyro.getYaw());
    inputs.yawVelocityRadPerSec = gyro.getAngularVel()[0];

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}

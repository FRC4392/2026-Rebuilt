// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeceiverRobotState;
import frc.robot.FieldConstants;
import frc.robot.lib.geometry.AllianceFlipUtil;
import frc.robot.subsystems.shooter.ShotCalculator.ShotParameters;
import frc.robot.subsystems.shooter.ShotCalculator.ShotType;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  @SuppressWarnings("unused")
  private final DeceiverRobotState robotState;

  private final Alert shooter1DisconnectedAlert =
      new Alert("Shooter Motor 1 Disconnected, expect reduced perfomance", AlertType.kError);
  private final Alert shooter2DisconnectedAlert =
      new Alert("Shooter Motor 2 Disconnected, expect reduced perfomance", AlertType.kError);
  private final Alert turretDisconnectedAlert =
      new Alert("Turret Motor Disconnected, Turret may not function", AlertType.kError);
  private final Alert hoodDisconnectedAlert =
      new Alert("Hood Motor Disconnected, hood may not function", AlertType.kError);
  private final Alert encoderDisconnectedAlert =
      new Alert(
          " Turret Absolute Encoder Disconnected, turret may be inaccurate", AlertType.kError);

  /** Creates a new Shooter. */
  public Shooter(ShooterIO IO, Supplier<AbsoluteEncoder> turrentEncoder) {
    shooterIO = IO;
    robotState = DeceiverRobotState.getInstance();

    shooterIO.setTurretAbsoluteEncoder(turrentEncoder.get());
  }

  @Override
  public void periodic() {
    if (this.getCurrentCommand() != null) {
      Logger.recordOutput("CurretCommand", this.getCurrentCommand().getName());
    } else {
      Logger.recordOutput("CurretCommand", "None");
    }
    shooterIO.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    shooter1DisconnectedAlert.set(!inputs.shooterMotor1Connected);
    shooter2DisconnectedAlert.set(!inputs.shooterMotor2Connected);
    turretDisconnectedAlert.set(!inputs.turretMotorConnected);
    hoodDisconnectedAlert.set(!inputs.hoodMotorConnected);
    encoderDisconnectedAlert.set(!inputs.turretAbsoluteEncoderConnected);
  }

  public void stop() {
    shooterIO.setShooter(Volts.of(0));
  }

  public Command setShooter(Supplier<Voltage> volts) {
    return this.runEnd(
        () -> {
          shooterIO.setShooter(volts.get());
        },
        this::stop);
  }

  public void setShooter(AngularVelocity velocity) {
    shooterIO.setShooter(velocity);
  }

  public Command setTurret(Supplier<Voltage> voltage) {
    return this.runEnd(
        () -> shooterIO.setShooter(voltage.get()), () -> shooterIO.setShooter(Volts.of(0)));
  }

  public Command setTurret(Angle angle) {
    return this.run(() -> shooterIO.setTurret(angle));
  }

  public Command shooterVelocityTuneCommand() {
    return this.runEnd(
        () -> setShooter(RotationsPerSecond.of(4500.0 / 60.0)),
        () -> setShooter(RotationsPerSecond.of(0)));
  }

  public Command setHood(Supplier<Voltage> volts) {
    return this.runEnd(
        () -> {
          shooterIO.setHood(volts.get());
        },
        () -> {
          shooterIO.setHood(Volts.of(0));
        });
  }

  public Command setHood(Angle angle) {
    return this.runOnce(
        () -> {
          shooterIO.setHood(angle);
        });
  }

  public Command setPose(Angle rotation, Angle hood, AngularVelocity speed) {
    return this.run(
        () -> {
          shooterIO.setHood(hood);
          shooterIO.setShooter(speed);
          // shooterIO.setTurret(rotation);
        });
  }

  public Command aimAtTarget(TargetLocation targetLocation) {
    return this.runEnd(
        () -> {
          Translation2d targetTranslation = getTargetTanslation(targetLocation);
          ShotType shotType = targetLocation == TargetLocation.Hub ? ShotType.Shot : ShotType.Pass;
          ShotParameters shotParameters =
              ShotCalculator.getInstance().getParameters(targetTranslation, shotType);
          shooterIO.setHood(shotParameters.hoodAngle());
          shooterIO.setShooter(shotParameters.flywheelSpeed());
          Rotation2d turretSetpoint =
              robotState.getRobotPose().getRotation().minus(shotParameters.turretAngle());

          Angle setpoint = turretSetpoint.getMeasure();

          if (setpoint.in(Degrees) < 0) {
            setpoint = setpoint.plus(Degrees.of(360));
          }

          Logger.recordOutput("Shot Angle", shotParameters.turretAngle());
          Logger.recordOutput("Turret Setpoint", setpoint);

          if (setpoint.in(Degrees) > 330) {
            setpoint = Degrees.of(330);
          }

          if (setpoint.in(Degrees) < 30) {
            setpoint = Degrees.of(30);
          }
          shooterIO.setTurret(setpoint);
          ShotCalculator.getInstance().clearLaunchingParameters();
        },
        () -> {});
  }

  private Translation2d getHubLocation() {
    return AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d());
  }

  private Translation2d getLeftPassLocation() {
    return AllianceFlipUtil.apply(FieldConstants.PassingPoint.leftPoint);
  }

  private Translation2d getRightPassLocation() {
    return AllianceFlipUtil.apply(FieldConstants.PassingPoint.rightPoint);
  }

  public enum TargetLocation {
    Hub,
    LeftPass,
    RightPass
  };

  private Translation2d getTargetTanslation(TargetLocation location) {
    switch (location) {
      case Hub:
        return getHubLocation();
      case LeftPass:
        return getLeftPassLocation();
      case RightPass:
        return getRightPassLocation();
      default:
        return new Translation2d();
    }
  }
}

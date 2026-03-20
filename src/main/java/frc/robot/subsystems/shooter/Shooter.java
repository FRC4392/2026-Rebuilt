// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
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
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

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

  // // PID stuff
  // private LoggedTunableNumber shooterKP =
  //     new LoggedTunableNumber("Shooter/kp", ShooterConstants.shooterKp);
  // private LoggedTunableNumber shooterKI =
  //     new LoggedTunableNumber("Shooter/ki", ShooterConstants.shooterKi);
  // private LoggedTunableNumber shooterKD =
  //     new LoggedTunableNumber("Shooter/kd", ShooterConstants.shooterKd);
  // private LoggedTunableNumber shooterKS =
  //     new LoggedTunableNumber("Shooter/ks", ShooterConstants.shooterKs);
  // private LoggedTunableNumber shooterKV =
  //     new LoggedTunableNumber("Shooter/kv", ShooterConstants.shooterKv);
  // private LoggedTunableNumber shooterKA =
  //     new LoggedTunableNumber("Shooter/ka", ShooterConstants.shooterKa);

  /** Creates a new Shooter. */
  public Shooter(ShooterIO IO) {
    shooterIO = IO;
    robotState = DeceiverRobotState.getInstance();
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    shooter1DisconnectedAlert.set(!inputs.shooterMotor1Connected);
    shooter2DisconnectedAlert.set(!inputs.shooterMotor2Connected);
    turretDisconnectedAlert.set(!inputs.turretMotorConnected);
    hoodDisconnectedAlert.set(!inputs.hoodMotorConnected);
    encoderDisconnectedAlert.set(!inputs.turretAbsoluteEncoderConnected);

    // if (shooterKP.hasChanged(hashCode())
    //     || shooterKI.hasChanged(hashCode())
    //     || shooterKD.hasChanged(hashCode())
    //     || shooterKS.hasChanged(hashCode())
    //     || shooterKV.hasChanged(hashCode())
    //     || shooterKA.hasChanged(hashCode())) {
    //   // double kp, double ki, double kd, double ks, double kv, double ka
    //   shooterIO.setPID(
    //       shooterKP.get(),
    //       shooterKI.get(),
    //       shooterKD.get(),
    //       shooterKS.get(),
    //       shooterKV.get(),
    //       shooterKA.get());
    // }
  }

  public void setShooter(Voltage volts) {
    shooterIO.setShooter(volts);
  }

  public void stop() {
    shooterIO.setShooter(Volts.of(0));
  }

  public Command setShooter(Supplier<Voltage> volts) {
    return this.runEnd(
        () -> {
          setShooter(volts.get());
        },
        this::stop);
  }

  public void setShooter(AngularVelocity velocity) {
    shooterIO.setShooter(velocity);
  }

  public Command runTestVoltage() {
    return this.runEnd(() -> setShooter(Volts.of(12)), () -> setShooter(Volts.of(0)));
  }

  public Command runTurret(Supplier<Voltage> voltage) {
    return this.runEnd(() -> setShooter(voltage.get()), () -> setShooter(Volts.of(0)));
  }

  public Command runTurret(Angle angle) {
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
    return this.runEnd(
        () -> {
          shooterIO.setHood(hood);
          shooterIO.setShooter(speed);
        },
        () -> {
          shooterIO.setShooter(RotationsPerSecond.of(0));
        });
  }

  public Command aimAtHub() {
    return this.runEnd(
        () -> {
          Pose2d robotLocation = robotState.getRobotPose();
          Pose2d shooterLocation = robotLocation.transformBy(ShooterTransorm);
          Logger.recordOutput("ShooterLocation", shooterLocation);


          Translation2d hubLocation = getTargetTanslation(TargetLocation.Hub);

          Translation2d resultingTranslation = hubLocation.minus(shooterLocation.getTranslation());
          Logger.recordOutput("Resulting Tanslation", resultingTranslation);

          Rotation2d shotAngle = resultingTranslation.getAngle();

          Rotation2d turretAngle =
              shotAngle.minus(robotState.getRobotPose().getRotation()).plus(Rotation2d.kPi);

          Logger.recordOutput("Target Angle", turretAngle);
        },
        () -> {});
  }

  private Translation2d getHubLocation() {
    return AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
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

  private Translation2d getTargetTanslation(TargetLocation location){
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

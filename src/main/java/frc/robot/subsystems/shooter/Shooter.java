// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  // PID stuff
  private LoggedTunableNumber shooterKP =
      new LoggedTunableNumber("Shooter/kp", ShooterConstants.shooterKp);
  private LoggedTunableNumber shooterKI =
      new LoggedTunableNumber("Shooter/ki", ShooterConstants.shooterKi);
  private LoggedTunableNumber shooterKD =
      new LoggedTunableNumber("Shooter/kd", ShooterConstants.shooterKd);
  private LoggedTunableNumber shooterKS =
      new LoggedTunableNumber("Shooter/ks", ShooterConstants.shooterKs);
  private LoggedTunableNumber shooterKV =
      new LoggedTunableNumber("Shooter/kv", ShooterConstants.shooterKv);
  private LoggedTunableNumber shooterKA =
      new LoggedTunableNumber("Shooter/ka", ShooterConstants.shooterKa);

  /** Creates a new Shooter. */
  public Shooter(ShooterIO IO) {
    shooterIO = IO;
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    if (shooterKP.hasChanged(hashCode())
        || shooterKI.hasChanged(hashCode())
        || shooterKD.hasChanged(hashCode())
        || shooterKS.hasChanged(hashCode())
        || shooterKV.hasChanged(hashCode())
        || shooterKA.hasChanged(hashCode())) {
      // double kp, double ki, double kd, double ks, double kv, double ka
      shooterIO.setPID(
          shooterKP.get(),
          shooterKI.get(),
          shooterKD.get(),
          shooterKS.get(),
          shooterKV.get(),
          shooterKA.get());
    }
  }

  public void setShooter(Voltage volts) {
    shooterIO.setShooter(volts);
  }

  public void setShooter(AngularVelocity velocity) {
    shooterIO.setShooter(velocity);
  }

  public Command runTestVoltage() {
    return this.runEnd(() -> setShooter(Volts.of(12)), () -> setShooter(Volts.of(0)));
  }

  public Command runTurret(DoubleSupplier voltage) {
    return this.runEnd(
        () -> setShooter(Volts.of(voltage.getAsDouble())), () -> setShooter(Volts.of(0)));
  }

  public Command shooterVelocityTuneCommand() {
    return this.runEnd(
        () -> setShooter(RotationsPerSecond.of(4500.0 / 60.0)),
        () -> setShooter(RotationsPerSecond.of(0)));
  }
}

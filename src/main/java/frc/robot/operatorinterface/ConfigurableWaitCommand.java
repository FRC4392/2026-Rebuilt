// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.operatorinterface;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ConfigurableWaitCommand extends Command {
  private Timer timer = new Timer();
  private final DoubleSupplier secondsSupplier;

  /** Creates a new ConfigurableWaitCommand. */
  public ConfigurableWaitCommand(DoubleSupplier seconds) {
    secondsSupplier = seconds;
  }

  public ConfigurableWaitCommand(Supplier<Time> timeSupplier) {
    secondsSupplier = () -> timeSupplier.get().in(Seconds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(secondsSupplier.getAsDouble());
  }
}

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Interface for all the inake subsystem IO */
public interface IntakeIO {

  /** Intake subsystem inputs */
  @AutoLog
  public static class IntakeIOInputs {
    public boolean extensionMotorConnected = false;
    public Angle extensionMotorPosition = Degrees.of(0);
    public AngularVelocity extensionMotorVelocity = DegreesPerSecond.of(0.0);
    public Voltage extensionMotorAppliedVolts = Volts.of(0.0);
    public Current extensionMotorCurrent = Amps.of(0.0);
    public Temperature extensionMotorTemp = Celsius.of(0.0);

    public boolean leftRollerMotorConnected = false;
    public Angle leftRollerMotorPosition = Degrees.of(0);
    public AngularVelocity leftRollerMotorVelocity = DegreesPerSecond.of(0.0);
    public Voltage leftRollerMotorAppliedVolts = Volts.of(0.0);
    public Current leftRollerMotorCurrent = Amps.of(0.0);
    public Temperature leftRollerMotorTemp = Celsius.of(0.0);

    public boolean rightRollerMotorConnected = false;
    public Angle rightRollerMotorPosition = Degrees.of(0);
    public AngularVelocity rightRollerMotorVelocity = DegreesPerSecond.of(0.0);
    public Voltage rightRollerMotorAppliedVolts = Volts.of(0.0);
    public Current rightRollerMotorCurrent = Amps.of(0.0);
    public Temperature rightRollerMotorTemp = Celsius.of(0.0);
  }

  /**
   * Update all inputs from subsystem
   *
   * @param inputs Inputs for subsystem to update
   */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /**
   * Set the voltage of the extension motor
   *
   * @param volts Voltage for the extension motor to run at
   */
  public default void setExtension(Voltage volts) {}

  /**
   * Set target angle of the extension drive system
   *
   * @param angle target angle for drive system
   */
  public default void setExtension(Angle angle) {}

  /**
   * Set the voltage of the roller motor
   *
   * @param volts Voltage for the roller motor to run at
   */
  public default void setRoller(Voltage volts) {}

  /**
   * Set the current for the roller motor
   *
   * @param current Current for the roller motor to run at
   */
  public default void setRoller(Current current) {}
}

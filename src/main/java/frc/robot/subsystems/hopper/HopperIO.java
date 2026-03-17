package frc.robot.subsystems.hopper;

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

public interface HopperIO {
  @AutoLog
  public static class HopperIOInputs {
    public boolean topMotorConnected = false;
    public Angle topMotorPosition = Degrees.of(0);
    public AngularVelocity topMotorVelocity = DegreesPerSecond.of(0.0);
    public Voltage topMotorAppliedVolts = Volts.of(0.0);
    public Current topMotorCurrent = Amps.of(0.0);
    public Temperature topMotorTemp = Celsius.of(0.0);

    public boolean bottomMotorConnected = false;
    public Angle bottomMotorPosition = Degrees.of(0);
    public AngularVelocity bottomMotorVelocity = DegreesPerSecond.of(0.0);
    public Voltage bottomMotorAppliedVolts = Volts.of(0.0);
    public Current bottomMotorCurrent = Amps.of(0.0);
    public Temperature bottomMotorTemp = Celsius.of(0.0);
  }

  public default void updateInputs(HopperIOInputs inputs) {}

  public default void setVoltage(Voltage volts) {}
}

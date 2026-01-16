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
    public boolean motorConnected = false;
    public Angle motorPosition = Degrees.of(0);
    public AngularVelocity motorVelocity = DegreesPerSecond.of(0.0);
    public Voltage motorAppliedVolts = Volts.of(0.0);
    public Current motorCurrent = Amps.of(0.0);
    public Temperature motorTemp = Celsius.of(0.0);
  }
}

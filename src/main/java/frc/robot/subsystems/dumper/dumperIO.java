package frc.robot.subsystems.dumper;

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

public interface DumperIO {
  @AutoLog
  public static class DumperIOInputs {
    public boolean dumperMotor1Connected = false;
    public Angle dumperMotor1Position = Degrees.of(0);
    public AngularVelocity dumperMotor1Velocity = DegreesPerSecond.of(0.0);
    public Voltage dumperMotor1AppliedVolts = Volts.of(0.0);
    public Current dumperMotor1Current = Amps.of(0.0);
    public Temperature dumperMotor1Temp = Celsius.of(0.0);

    public boolean dumperMotor2Connected = false;
    public Angle dumperMotor2Position = Degrees.of(0);
    public AngularVelocity dumperMotor2Velocity = DegreesPerSecond.of(0.0);
    public Voltage dumperMotor2AppliedVolts = Volts.of(0.0);
    public Current dumperMotor2Current = Amps.of(0.0);
    public Temperature dumperMotor2Temp = Celsius.of(0.0);

    public boolean dumperMotor3Connected = false;
    public Angle dumperMotor3Position = Degrees.of(0);
    public AngularVelocity dumperMotor3Velocity = DegreesPerSecond.of(0.0);
    public Voltage dumperMotor3AppliedVolts = Volts.of(0.0);
    public Current dumperMotor3Current = Amps.of(0.0);
    public Temperature dumperMotor3Temp = Celsius.of(0.0);

    public boolean dumperMotor4Connected = false;
    public Angle dumperMotor4Position = Degrees.of(0);
    public AngularVelocity dumperMotor4Velocity = DegreesPerSecond.of(0.0);
    public Voltage dumperMotor4AppliedVolts = Volts.of(0.0);
    public Current dumperMotor4Current = Amps.of(0.0);
    public Temperature dumperMotor4Temp = Celsius.of(0.0);
  }

  public default void updateInputs(DumperIOInputs inputs) {}

  public default void setVoltage(Voltage volts) {}
}

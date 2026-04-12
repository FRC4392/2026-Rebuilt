package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean shooterMotor1Connected = false;
    public Angle shooterMotor1Position = Degrees.of(0);
    public AngularVelocity shooterMotor1Velocity = DegreesPerSecond.of(0.0);
    public Voltage shooterMotor1AppliedVolts = Volts.of(0.0);
    public Current shooterMotor1StatorCurrent = Amps.of(0.0);
    public Temperature shooterMotor1Temp = Celsius.of(0.0);
    public Current shooterMotor1SupplyCurrent = Amps.of(0.0);

    public boolean shooterMotor2Connected = false;
    public Angle shooterMotor2Position = Degrees.of(0);
    public AngularVelocity shooterMotor2Velocity = DegreesPerSecond.of(0.0);
    public Voltage shooterMotor2AppliedVolts = Volts.of(0.0);
    public Current shooterMotor2StatorCurrent = Amps.of(0.0);
    public Temperature shooterMotor2Temp = Celsius.of(0.0);
    public Current shooterMotor2SupplyCurrent = Amps.of(0.0);

    public boolean turretMotorConnected = false;
    public Angle turretMotorPosition = Degrees.of(0);
    public AngularVelocity turretMotorVelocity = DegreesPerSecond.of(0.0);
    public Voltage turretMotorAppliedVolts = Volts.of(0.0);
    public Current turretMotorStatorCurrent = Amps.of(0.0);
    public Temperature turretMotorTemp = Celsius.of(0.0);
    public Current turretMotorSupplyCurrent = Amps.of(0.0);

    public boolean hoodMotorConnected = false;
    public Angle hoodMotorPosition = Degrees.of(0);
    public AngularVelocity hoodMotorVelocity = DegreesPerSecond.of(0.0);
    public Voltage hoodMotorAppliedVolts = Volts.of(0.0);
    public Current hoodMotorCurrent = Amps.of(0.0);
    public Temperature hoodMotorTemp = Celsius.of(0.0);

    public Angle turretAbsoluteAngle = Degrees.of(0);
    public boolean turretAbsoluteEncoderConnected = false;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setShooter(Voltage volts) {}

  public default void setShooter(AngularVelocity velocity) {}
  ;

  public default void setTurret(Voltage volts) {}
  ;

  public default void setTurret(Angle angle) {}
  ;

  // public default void setPID(double kp, double ki, double kd, double ks, double kv, double ka) {}
  ;

  public default void setHood(Voltage volts) {}
  ;

  public default void setHood(Angle angle) {}
  ;

  public default void setTurretAbsoluteEncoder(AbsoluteEncoder encoder) {}
  ;
}

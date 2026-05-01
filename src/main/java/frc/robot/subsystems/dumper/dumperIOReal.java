package frc.robot.subsystems.dumper;

import static frc.robot.lib.util.PhoenixUtil.tryUntilOk;
import static frc.robot.subsystems.dumper.DumperConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class DumperIOReal implements DumperIO {
  // Motors
  public final TalonFX dumperMotor1;
  public final TalonFX dumperMotor2;
  public final TalonFX dumperMotor3;
  public final TalonFX dumperMotor4;

  // Conrtol Requests
  private final VoltageOut dumperMotor1VoltageRequest = new VoltageOut(0);
  private final VoltageOut dumperMotor2VoltageRequest = new VoltageOut(0);
  private final VoltageOut dumperMotor3VoltageRequest = new VoltageOut(0);
  private final VoltageOut dumperMotor4VoltageRequest = new VoltageOut(0);

  // Status Signals
  private final StatusSignal<Angle> dumperMotor1Position;
  private final StatusSignal<AngularVelocity> dumperMotor1Velocity;
  private final StatusSignal<Voltage> dumperMotor1Voltage;
  private final StatusSignal<Current> dumperMotor1Current;
  private final StatusSignal<Temperature> dumperMotor1Temperature;

  private final StatusSignal<Angle> dumperMotor2Position;
  private final StatusSignal<AngularVelocity> dumperMotor2Velocity;
  private final StatusSignal<Voltage> dumperMotor2Voltage;
  private final StatusSignal<Current> dumperMotor2Current;
  private final StatusSignal<Temperature> dumperMotor2Temperature;

  private final StatusSignal<Angle> dumperMotor3Position;
  private final StatusSignal<AngularVelocity> dumperMotor3Velocity;
  private final StatusSignal<Voltage> dumperMotor3Voltage;
  private final StatusSignal<Current> dumperMotor3Current;
  private final StatusSignal<Temperature> dumperMotor3Temperature;

  private final StatusSignal<Angle> dumperMotor4Position;
  private final StatusSignal<AngularVelocity> dumperMotor4Velocity;
  private final StatusSignal<Voltage> dumperMotor4Voltage;
  private final StatusSignal<Current> dumperMotor4Current;
  private final StatusSignal<Temperature> dumperMotor4Temperature;

  // Debouncers
  private final Debouncer dumperMotor1ConnectDebouncer = new Debouncer(.25);
  private final Debouncer dumperMotor2ConnectDebouncer = new Debouncer(.25);
  private final Debouncer dumperMotor3ConnectDebouncer = new Debouncer(.25);
  private final Debouncer dumperMotor4ConnectDebouncer = new Debouncer(.25);

  public DumperIOReal() {
    dumperMotor1 = new TalonFX(dumperMotor1CanID);

    TalonFXConfiguration dumperMotor1Configuration =
        new TalonFXConfiguration()
            .withAudio(
                new AudioConfigs()
                    .withAllowMusicDurDisable(enableBeeps)
                    .withBeepOnBoot(enableBeeps)
                    .withBeepOnConfig(enableBeeps))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(dumperMotor1StatorCurrentLimit)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(dumperMotor1MotorSupplyLimitHigh)
                    .withSupplyCurrentLowerLimit(dumperMotor1MotorSupplyLimitLow)
                    .withSupplyCurrentLowerTime(dumperMotor1SupplyCurrentLowerTime)
                    .withSupplyCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(dumperMotor1MotorReduction))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(dumperMotor1Inverted)
                    .withNeutralMode(dumperMotor1NeutralMode))
            .withSlot0(
                new Slot0Configs()
                    .withKP(dumperMotor1Kp)
                    .withKI(dumperMotor1Ki)
                    .withKD(dumperMotor1Kd)
                    .withKG(0)
                    .withKV(dumperMotor1Kv)
                    .withKS(dumperMotor1Ks)
                    .withKA(dumperMotor1Ka))
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(dumperMotor1StatorCurrentLimit)
                    .withPeakReverseTorqueCurrent(dumperMotor1StatorCurrentLimit.unaryMinus()));

    tryUntilOk(5, () -> dumperMotor1.getConfigurator().apply(dumperMotor1Configuration, 0.25));

    // Motor 2
    dumperMotor2 = new TalonFX(dumperMotor2CanID);

    TalonFXConfiguration dumperMotor2Configuration =
        new TalonFXConfiguration()
            .withAudio(
                new AudioConfigs()
                    .withAllowMusicDurDisable(enableBeeps)
                    .withBeepOnBoot(enableBeeps)
                    .withBeepOnConfig(enableBeeps))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(dumperMotor2StatorCurrentLimit)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(dumperMotor2MotorSupplyLimitHigh)
                    .withSupplyCurrentLowerLimit(dumperMotor2MotorSupplyLimitLow)
                    .withSupplyCurrentLowerTime(dumperMotor2SupplyCurrentLowerTime)
                    .withSupplyCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(dumperMotor2MotorReduction))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(dumperMotor2Inverted)
                    .withNeutralMode(dumperMotor2NeutralMode))
            .withSlot0(
                new Slot0Configs()
                    .withKP(dumperMotor2Kp)
                    .withKI(dumperMotor2Ki)
                    .withKD(dumperMotor2Kd)
                    .withKG(0)
                    .withKV(dumperMotor2Kv)
                    .withKS(dumperMotor2Ks)
                    .withKA(dumperMotor2Ka))
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(dumperMotor2StatorCurrentLimit)
                    .withPeakReverseTorqueCurrent(dumperMotor2StatorCurrentLimit.unaryMinus()));

    tryUntilOk(5, () -> dumperMotor2.getConfigurator().apply(dumperMotor2Configuration, 0.25));

    // Motor 3
    dumperMotor3 = new TalonFX(dumperMotor3CanID);

    TalonFXConfiguration dumperMotor3Configuration =
        new TalonFXConfiguration()
            .withAudio(
                new AudioConfigs()
                    .withAllowMusicDurDisable(enableBeeps)
                    .withBeepOnBoot(enableBeeps)
                    .withBeepOnConfig(enableBeeps))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(dumperMotor3StatorCurrentLimit)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(dumperMotor3MotorSupplyLimitHigh)
                    .withSupplyCurrentLowerLimit(dumperMotor3MotorSupplyLimitLow)
                    .withSupplyCurrentLowerTime(dumperMotor3SupplyCurrentLowerTime)
                    .withSupplyCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(dumperMotor3MotorReduction))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(dumperMotor3Inverted)
                    .withNeutralMode(dumperMotor3NeutralMode))
            .withSlot0(
                new Slot0Configs()
                    .withKP(dumperMotor3Kp)
                    .withKI(dumperMotor3Ki)
                    .withKD(dumperMotor3Kd)
                    .withKG(0)
                    .withKV(dumperMotor3Kv)
                    .withKS(dumperMotor3Ks)
                    .withKA(dumperMotor3Ka))
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(dumperMotor3StatorCurrentLimit)
                    .withPeakReverseTorqueCurrent(dumperMotor3StatorCurrentLimit.unaryMinus()));

    tryUntilOk(5, () -> dumperMotor3.getConfigurator().apply(dumperMotor3Configuration, 0.25));

    // Motor 4
    dumperMotor4 = new TalonFX(dumperMotor4CanID);

    TalonFXConfiguration dumperMotor4Configuration =
        new TalonFXConfiguration()
            .withAudio(
                new AudioConfigs()
                    .withAllowMusicDurDisable(enableBeeps)
                    .withBeepOnBoot(enableBeeps)
                    .withBeepOnConfig(enableBeeps))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(dumperMotor4StatorCurrentLimit)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(dumperMotor4MotorSupplyLimitHigh)
                    .withSupplyCurrentLowerLimit(dumperMotor4MotorSupplyLimitLow)
                    .withSupplyCurrentLowerTime(dumperMotor4SupplyCurrentLowerTime)
                    .withSupplyCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(dumperMotor4MotorReduction))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(dumperMotor4Inverted)
                    .withNeutralMode(dumperMotor4NeutralMode))
            .withSlot0(
                new Slot0Configs()
                    .withKP(dumperMotor4Kp)
                    .withKI(dumperMotor4Ki)
                    .withKD(dumperMotor4Kd)
                    .withKG(0)
                    .withKV(dumperMotor4Kv)
                    .withKS(dumperMotor4Ks)
                    .withKA(dumperMotor4Ka))
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(dumperMotor4StatorCurrentLimit)
                    .withPeakReverseTorqueCurrent(dumperMotor4StatorCurrentLimit.unaryMinus()));

    tryUntilOk(5, () -> dumperMotor4.getConfigurator().apply(dumperMotor4Configuration, 0.25));

    dumperMotor1Position = dumperMotor1.getPosition();
    dumperMotor1Velocity = dumperMotor1.getVelocity();
    dumperMotor1Voltage = dumperMotor1.getMotorVoltage();
    dumperMotor1Current = dumperMotor1.getStatorCurrent();
    dumperMotor1Temperature = dumperMotor1.getDeviceTemp();

    dumperMotor2Position = dumperMotor2.getPosition();
    dumperMotor2Velocity = dumperMotor2.getVelocity();
    dumperMotor2Voltage = dumperMotor2.getMotorVoltage();
    dumperMotor2Current = dumperMotor2.getStatorCurrent();
    dumperMotor2Temperature = dumperMotor2.getDeviceTemp();

    dumperMotor3Position = dumperMotor3.getPosition();
    dumperMotor3Velocity = dumperMotor3.getVelocity();
    dumperMotor3Voltage = dumperMotor3.getMotorVoltage();
    dumperMotor3Current = dumperMotor3.getStatorCurrent();
    dumperMotor3Temperature = dumperMotor3.getDeviceTemp();

    dumperMotor4Position = dumperMotor4.getPosition();
    dumperMotor4Velocity = dumperMotor4.getVelocity();
    dumperMotor4Voltage = dumperMotor4.getMotorVoltage();
    dumperMotor4Current = dumperMotor4.getStatorCurrent();
    dumperMotor4Temperature = dumperMotor4.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        dumperMotor1Position,
        dumperMotor1Velocity,
        dumperMotor1Voltage,
        dumperMotor1Current,
        dumperMotor1Temperature,
        dumperMotor2Position,
        dumperMotor2Velocity,
        dumperMotor2Voltage,
        dumperMotor2Current,
        dumperMotor2Temperature,
        dumperMotor3Position,
        dumperMotor3Velocity,
        dumperMotor3Voltage,
        dumperMotor3Current,
        dumperMotor3Temperature,
        dumperMotor4Position,
        dumperMotor4Velocity,
        dumperMotor4Voltage,
        dumperMotor4Current,
        dumperMotor4Temperature);
    ParentDevice.optimizeBusUtilizationForAll(
        dumperMotor1, dumperMotor2, dumperMotor3, dumperMotor4);

    dumperMotor1VoltageRequest.EnableFOC = true;
  }

  @Override
  public void updateInputs(DumperIOInputs inputs) {
    // Motor 1
    var dumperMotor1Status =
        BaseStatusSignal.refreshAll(
            dumperMotor1Position,
            dumperMotor1Velocity,
            dumperMotor1Voltage,
            dumperMotor1Current,
            dumperMotor1Temperature);

    inputs.dumperMotor1Connected =
        dumperMotor1ConnectDebouncer.calculate(dumperMotor1Status.isOK());
    inputs.dumperMotor1Position = dumperMotor1Position.getValue();
    inputs.dumperMotor1Velocity = dumperMotor1Velocity.getValue();
    inputs.dumperMotor1AppliedVolts = dumperMotor1Voltage.getValue();
    inputs.dumperMotor1Current = dumperMotor1Current.getValue();
    inputs.dumperMotor1Temp = dumperMotor1Temperature.getValue();

    // Motor 2
    var dumperMotor2Status =
        BaseStatusSignal.refreshAll(
            dumperMotor2Position,
            dumperMotor2Velocity,
            dumperMotor2Voltage,
            dumperMotor2Current,
            dumperMotor2Temperature);

    inputs.dumperMotor2Connected =
        dumperMotor2ConnectDebouncer.calculate(dumperMotor2Status.isOK());
    inputs.dumperMotor2Position = dumperMotor2Position.getValue();
    inputs.dumperMotor2Velocity = dumperMotor2Velocity.getValue();
    inputs.dumperMotor2AppliedVolts = dumperMotor2Voltage.getValue();
    inputs.dumperMotor2Current = dumperMotor2Current.getValue();
    inputs.dumperMotor2Temp = dumperMotor2Temperature.getValue();

    // Motor 3
    var dumperMotor3Status =
        BaseStatusSignal.refreshAll(
            dumperMotor3Position,
            dumperMotor3Velocity,
            dumperMotor3Voltage,
            dumperMotor3Current,
            dumperMotor3Temperature);

    inputs.dumperMotor3Connected =
        dumperMotor3ConnectDebouncer.calculate(dumperMotor3Status.isOK());
    inputs.dumperMotor3Position = dumperMotor3Position.getValue();
    inputs.dumperMotor3Velocity = dumperMotor3Velocity.getValue();
    inputs.dumperMotor3AppliedVolts = dumperMotor3Voltage.getValue();
    inputs.dumperMotor3Current = dumperMotor3Current.getValue();
    inputs.dumperMotor3Temp = dumperMotor3Temperature.getValue();

    // Motor 4
    var dumperMotor4Status =
        BaseStatusSignal.refreshAll(
            dumperMotor4Position,
            dumperMotor4Velocity,
            dumperMotor4Voltage,
            dumperMotor4Current,
            dumperMotor4Temperature);

    inputs.dumperMotor4Connected =
        dumperMotor4ConnectDebouncer.calculate(dumperMotor4Status.isOK());
    inputs.dumperMotor4Position = dumperMotor4Position.getValue();
    inputs.dumperMotor4Velocity = dumperMotor4Velocity.getValue();
    inputs.dumperMotor4AppliedVolts = dumperMotor4Voltage.getValue();
    inputs.dumperMotor4Current = dumperMotor4Current.getValue();
    inputs.dumperMotor4Temp = dumperMotor4Temperature.getValue();
  }

  @Override
  public void setVoltage(Voltage volts) {
    dumperMotor1.setControl(dumperMotor1VoltageRequest.withOutput(volts));
    dumperMotor2.setControl(dumperMotor2VoltageRequest.withOutput(volts));
    dumperMotor3.setControl(dumperMotor3VoltageRequest.withOutput(volts));
    dumperMotor4.setControl(dumperMotor4VoltageRequest.withOutput(volts));
  }
}

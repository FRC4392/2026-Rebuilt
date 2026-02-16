package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

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

public class ClimberIOReal implements ClimberIO {
  // Motors
  public final TalonFX climberMotor;

  // Conrtol Requests
  private final VoltageOut voltageRequest = new VoltageOut(0);

  // Status Signals
  private final StatusSignal<Angle> climberPosition;
  private final StatusSignal<AngularVelocity> climberVelocity;
  private final StatusSignal<Voltage> climberVoltage;
  private final StatusSignal<Current> climberCurrent;
  private final StatusSignal<Temperature> climberTemperatre;

  // Debouncers
  private final Debouncer motorConnectDebouncer = new Debouncer(.25);

  public ClimberIOReal() {
    climberMotor = new TalonFX(climberMotorCanID);

    TalonFXConfiguration climberConfiguration =
        new TalonFXConfiguration()
            .withAudio(
                new AudioConfigs()
                    .withAllowMusicDurDisable(enableBeeps)
                    .withBeepOnBoot(enableBeeps)
                    .withBeepOnConfig(enableBeeps))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(climberStatorCurrentLimit)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(climberMotorSupplyLimitHigh)
                    .withSupplyCurrentLowerLimit(climberMotorSupplyLimitLow)
                    .withSupplyCurrentLowerTime(climberSupplyCurrentLowerTime)
                    .withSupplyCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(climberMotorReduction))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(climberInverted)
                    .withNeutralMode(climberNeutralMode))
            .withSlot0(
                new Slot0Configs()
                    .withKP(climberKp)
                    .withKI(climberKi)
                    .withKD(climberKd)
                    .withKG(0)
                    .withKV(climberKv)
                    .withKS(climberKs)
                    .withKA(climberKa))
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(climberStatorCurrentLimit)
                    .withPeakReverseTorqueCurrent(climberStatorCurrentLimit.unaryMinus()));

    tryUntilOk(5, () -> climberMotor.getConfigurator().apply(climberConfiguration, 0.25));

    climberPosition = climberMotor.getPosition();
    climberVelocity = climberMotor.getVelocity();
    climberVoltage = climberMotor.getMotorVoltage();
    climberCurrent = climberMotor.getStatorCurrent();
    climberTemperatre = climberMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, climberPosition, climberVelocity, climberVoltage, climberCurrent, climberTemperatre);
    ParentDevice.optimizeBusUtilizationForAll(climberMotor);

    voltageRequest.EnableFOC = true;
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    var motorStatus =
        BaseStatusSignal.refreshAll(
            climberPosition, climberVelocity, climberVoltage, climberCurrent, climberTemperatre);

    inputs.motorConnected = motorConnectDebouncer.calculate(motorStatus.isOK());
    inputs.motorPosition = climberPosition.getValue();
    inputs.motorVelocity = climberVelocity.getValue();
    inputs.motorAppliedVolts = climberVoltage.getValue();
    inputs.motorCurrent = climberCurrent.getValue();
    inputs.motorTemp = climberTemperatre.getValue();
  }

  @Override
  public void setVoltage(Voltage volts) {
    climberMotor.setControl(voltageRequest.withOutput(volts));
  }
}

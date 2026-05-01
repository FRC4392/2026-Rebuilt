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
  public final TalonFX dumperMotor;

  // Conrtol Requests
  private final VoltageOut voltageRequest = new VoltageOut(0);

  // Status Signals
  private final StatusSignal<Angle> dumperPosition;
  private final StatusSignal<AngularVelocity> dumperVelocity;
  private final StatusSignal<Voltage> dumperVoltage;
  private final StatusSignal<Current> dumperCurrent;
  private final StatusSignal<Temperature> dumperTemperatre;

  // Debouncers
  private final Debouncer motorConnectDebouncer = new Debouncer(.25);

  public DumperIOReal() {
    dumperMotor = new TalonFX(dumperMotorCanID);

    TalonFXConfiguration dumperConfiguration =
        new TalonFXConfiguration()
            .withAudio(
                new AudioConfigs()
                    .withAllowMusicDurDisable(enableBeeps)
                    .withBeepOnBoot(enableBeeps)
                    .withBeepOnConfig(enableBeeps))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(dumperStatorCurrentLimit)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(dumperMotorSupplyLimitHigh)
                    .withSupplyCurrentLowerLimit(dumperMotorSupplyLimitLow)
                    .withSupplyCurrentLowerTime(dumperSupplyCurrentLowerTime)
                    .withSupplyCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(dumperMotorReduction))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(dumperInverted)
                    .withNeutralMode(dumperNeutralMode))
            .withSlot0(
                new Slot0Configs()
                    .withKP(dumperKp)
                    .withKI(dumperKi)
                    .withKD(dumperKd)
                    .withKG(0)
                    .withKV(dumperKv)
                    .withKS(dumperKs)
                    .withKA(dumperKa))
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(dumperStatorCurrentLimit)
                    .withPeakReverseTorqueCurrent(dumperStatorCurrentLimit.unaryMinus()));

    tryUntilOk(5, () -> dumperMotor.getConfigurator().apply(dumperConfiguration, 0.25));

    dumperPosition = dumperMotor.getPosition();
    dumperVelocity = dumperMotor.getVelocity();
    dumperVoltage = dumperMotor.getMotorVoltage();
    dumperCurrent = dumperMotor.getStatorCurrent();
    dumperTemperatre = dumperMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, dumperPosition, dumperVelocity, dumperVoltage, dumperCurrent, dumperTemperatre);
    ParentDevice.optimizeBusUtilizationForAll(dumperMotor);

    voltageRequest.EnableFOC = true;
  }

  @Override
  public void updateInputs(DumperIOInputs inputs) {
    var motorStatus =
        BaseStatusSignal.refreshAll(
            dumperPosition, dumperVelocity, dumperVoltage, dumperCurrent, dumperTemperatre);

    inputs.motorConnected = motorConnectDebouncer.calculate(motorStatus.isOK());
    inputs.motorPosition = dumperPosition.getValue();
    inputs.motorVelocity = dumperVelocity.getValue();
    inputs.motorAppliedVolts = dumperVoltage.getValue();
    inputs.motorCurrent = dumperCurrent.getValue();
    inputs.motorTemp = dumperTemperatre.getValue();
  }

  @Override
  public void setVoltage(Voltage volts) {
    dumperMotor.setControl(voltageRequest.withOutput(volts));
  }
}

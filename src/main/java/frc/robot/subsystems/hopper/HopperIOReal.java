package frc.robot.subsystems.hopper;

import static frc.robot.subsystems.hopper.HopperConstants.*;
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

public class HopperIOReal implements HopperIO {
  // Motors
  public final TalonFX hopperMotor;

  // Conrtol Requests
  private final VoltageOut voltageRequest = new VoltageOut(0);

  // Status Signals
  private final StatusSignal<Angle> hopperPosition;
  private final StatusSignal<AngularVelocity> hopperVelocity;
  private final StatusSignal<Voltage> hopperVoltage;
  private final StatusSignal<Current> hopperCurrent;
  private final StatusSignal<Temperature> hopperTemperatre;

  // Debouncers
  private final Debouncer motorConnectDebouncer = new Debouncer(.25);

  public HopperIOReal() {
    hopperMotor = new TalonFX(hopperMotorCanID);

    TalonFXConfiguration hopperConfiguration =
        new TalonFXConfiguration()
            .withAudio(
                new AudioConfigs()
                    .withAllowMusicDurDisable(enableBeeps)
                    .withBeepOnBoot(enableBeeps)
                    .withBeepOnConfig(enableBeeps))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(hopperStatorCurrentLimit)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(hopperMotorSupplyLimitHigh)
                    .withSupplyCurrentLowerLimit(hopperMotorSupplyLimitLow)
                    .withSupplyCurrentLowerTime(hopperSupplyCurrentLowerTime)
                    .withSupplyCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(hopperMotorReduction))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(hopperInverted)
                    .withNeutralMode(hopperNeutralMode))
            .withSlot0(
                new Slot0Configs()
                    .withKP(hopperKp)
                    .withKI(hopperKi)
                    .withKD(hopperKd)
                    .withKG(0)
                    .withKV(hopperKv)
                    .withKS(hopperKs)
                    .withKA(hopperKa))
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(hopperStatorCurrentLimit)
                    .withPeakReverseTorqueCurrent(hopperStatorCurrentLimit.unaryMinus()));

    tryUntilOk(5, () -> hopperMotor.getConfigurator().apply(hopperConfiguration, 0.25));

    hopperPosition = hopperMotor.getPosition();
    hopperVelocity = hopperMotor.getVelocity();
    hopperVoltage = hopperMotor.getMotorVoltage();
    hopperCurrent = hopperMotor.getStatorCurrent();
    hopperTemperatre = hopperMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, hopperPosition, hopperVelocity, hopperVoltage, hopperCurrent, hopperTemperatre);
    ParentDevice.optimizeBusUtilizationForAll(hopperMotor);

    voltageRequest.EnableFOC = true;
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    var motorStatus =
        BaseStatusSignal.refreshAll(
            hopperPosition, hopperVelocity, hopperVoltage, hopperCurrent, hopperTemperatre);

    inputs.motorConnected = motorConnectDebouncer.calculate(motorStatus.isOK());
    inputs.motorPosition = hopperPosition.getValue();
    inputs.motorVelocity = hopperVelocity.getValue();
    inputs.motorAppliedVolts = hopperVoltage.getValue();
    inputs.motorCurrent = hopperCurrent.getValue();
    inputs.motorTemp = hopperTemperatre.getValue();
  }

  @Override
  public void setHopper(Voltage volts) {
    hopperMotor.setControl(voltageRequest.withOutput(volts));
  }
}

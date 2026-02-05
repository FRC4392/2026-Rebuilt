package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.*;
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

public class IndexerIOReal implements IndexerIO {
    //Motors
    public final TalonFX indexerMotor;

    //Conrtol Requests
    private final VoltageOut voltageRequest = new VoltageOut(0);

    //Status Signals
    private final StatusSignal<Angle> indexerPosition;
    private final StatusSignal<AngularVelocity> indexerVelocity;
    private final StatusSignal<Voltage> indexerVoltage;
    private final StatusSignal<Current> indexerCurrent;
    private final StatusSignal<Temperature> indexerTemperatre;

    //Debouncers
    private final Debouncer motorConnectDebouncer = new Debouncer(.25);

    public IndexerIOReal(){
        indexerMotor = new TalonFX(indexerMotorCanID);

        TalonFXConfiguration indexerConfiguration =
      new TalonFXConfiguration()
          .withAudio(
              new AudioConfigs()
                  .withAllowMusicDurDisable(enableBeeps)
                  .withBeepOnBoot(enableBeeps)
                  .withBeepOnConfig(enableBeeps))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(indexerStatorCurrentLimit)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(indexerMotorSupplyLimitHigh)
                  .withSupplyCurrentLowerLimit(indexerMotorSupplyLimitLow)
                  .withSupplyCurrentLowerTime(indexerSupplyCurrentLowerTime)
                  .withSupplyCurrentLimitEnable(true))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  .withSensorToMechanismRatio(indexerMotorReduction))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(indexerInverted)
                  .withNeutralMode(indexerNeutralMode))
          .withSlot0(
              new Slot0Configs()
                  .withKP(indexerKp)
                  .withKI(indexerKi)
                  .withKD(indexerKd)
                  .withKG(0)
                  .withKV(indexerKv)
                  .withKS(indexerKs)
                  .withKA(indexerKa))
          .withTorqueCurrent(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(indexerStatorCurrentLimit)
                  .withPeakReverseTorqueCurrent(indexerStatorCurrentLimit.unaryMinus()));

        tryUntilOk(5, ()-> indexerMotor.getConfigurator().apply(indexerConfiguration, 0.25));

        indexerPosition = indexerMotor.getPosition();
        indexerVelocity = indexerMotor.getVelocity();
        indexerVoltage = indexerMotor.getMotorVoltage();
        indexerCurrent = indexerMotor.getStatorCurrent();
        indexerTemperatre = indexerMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, indexerPosition, indexerVelocity, indexerVoltage, indexerCurrent, indexerTemperatre);
        ParentDevice.optimizeBusUtilizationForAll(indexerMotor);

        voltageRequest.EnableFOC = true;
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        var motorStatus = BaseStatusSignal.refreshAll(indexerPosition, indexerVelocity, indexerVoltage, indexerCurrent, indexerTemperatre);

        inputs.motorConnected = motorConnectDebouncer.calculate(motorStatus.isOK());
        inputs.motorPosition = indexerPosition.getValue();
        inputs.motorVelocity = indexerVelocity.getValue();
        inputs.motorAppliedVolts = indexerVoltage.getValue();
        inputs.motorCurrent = indexerCurrent.getValue();
        inputs.motorTemp = indexerTemperatre.getValue();
    }

    @Override
    public void setIndexer(Voltage volts) {
        indexerMotor.setControl(voltageRequest.withOutput(volts));
    }
}

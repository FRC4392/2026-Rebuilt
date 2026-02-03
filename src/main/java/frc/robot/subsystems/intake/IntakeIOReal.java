package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;
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

public class IntakeIOReal implements IntakeIO {
    //Motors
    private final TalonFX intakeMotor;

    //Control Requests
    private final VoltageOut voltageRequest = new VoltageOut(0);

    //Status Signals
    private final StatusSignal<Angle> intakePosition;
    private final StatusSignal<AngularVelocity> intakeVelocity;
    private final StatusSignal<Voltage> intakeVoltage;
    private final StatusSignal<Current> intakeCurrent;
    private final StatusSignal<Temperature> intakeTemperature;

    //Debouncers
    private final Debouncer motorConnectedDebouncer = new Debouncer(.25);

    public IntakeIOReal(){
        intakeMotor = new TalonFX(intakeMotorCanID);

        TalonFXConfiguration intakeConfiguration =
      new TalonFXConfiguration()
          .withAudio(
              new AudioConfigs()
                  .withAllowMusicDurDisable(enableBeeps)
                  .withBeepOnBoot(enableBeeps)
                  .withBeepOnConfig(enableBeeps))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(intakeStatorCurrentLimit)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(intakeMotorSupplyLimitHigh)
                  .withSupplyCurrentLowerLimit(intakeMotorSupplyLimitLow)
                  .withSupplyCurrentLowerTime(intakeSupplyCurrentLowerTime)
                  .withSupplyCurrentLimitEnable(true))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  .withSensorToMechanismRatio(intakeMotorReduction))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(intakeInverted)
                  .withNeutralMode(intakeNeutralMode))
          .withSlot0(
              new Slot0Configs()
                  .withKP(intakeKp)
                  .withKI(intakeKi)
                  .withKD(intakeKd)
                  .withKG(0)
                  .withKV(intakeKv)
                  .withKS(intakeKs)
                  .withKA(intakeKa))
          .withTorqueCurrent(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(intakeStatorCurrentLimit)
                  .withPeakReverseTorqueCurrent(intakeStatorCurrentLimit.unaryMinus()));

        tryUntilOk(5, ()->intakeMotor.getConfigurator().apply(intakeConfiguration, 0.25));

        intakePosition = intakeMotor.getPosition();
        intakeVelocity = intakeMotor.getVelocity();
        intakeVoltage = intakeMotor.getMotorVoltage();
        intakeCurrent = intakeMotor.getStatorCurrent();
        intakeTemperature = intakeMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, intakePosition, intakeVelocity, intakeVoltage, intakeCurrent, intakeTemperature);
        ParentDevice.optimizeBusUtilizationForAll(intakeMotor);

        voltageRequest.EnableFOC = true;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs){
        var motorStatus = BaseStatusSignal.refreshAll(intakePosition, intakeVelocity, intakeVoltage, intakeCurrent, intakeTemperature);

        inputs.motorConnected = motorConnectedDebouncer.calculate(motorStatus.isOK());
        inputs.motorPosition = intakePosition.getValue();
        inputs.motorVelocity = intakeVelocity.getValue();
        inputs.motorAppliedVolts = intakeVoltage.getValue();
        inputs.motorCurrent = intakeCurrent.getValue();
        inputs.motorTemp = intakeTemperature.getValue();
    }

    @Override
    public void setIntake(Voltage volts) {
        intakeMotor.setControl(voltageRequest.withOutput(volts));
    }

}

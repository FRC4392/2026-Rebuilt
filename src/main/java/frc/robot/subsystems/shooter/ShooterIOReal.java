package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;
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
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

public class ShooterIOReal implements ShooterIO {
    //Motors
    public final TalonFX shooterMotor1;
    public final TalonFX shooterMotor2;
    public final TalonFX turretMotor;
    public final TalonFX hoodMotor;

    //Conrtol Requests
    private final VoltageOut voltageRequest = new VoltageOut(0);

    //Status Signals
    private final StatusSignal<Angle> shooterMotor1Position;
    private final StatusSignal<AngularVelocity> shooterMotor1Velocity;
    private final StatusSignal<Voltage> shooterMotor1Voltage;
    private final StatusSignal<Current> shooterMotor1Current;
    private final StatusSignal<Temperature> shooterMotor1Temperatre;

    private final StatusSignal<Angle> shooterMotor2Position;
    private final StatusSignal<AngularVelocity> shooterMotor2Velocity;
    private final StatusSignal<Voltage> shooterMotor2Voltage;
    private final StatusSignal<Current> shooterMotor2Current;
    private final StatusSignal<Temperature> shooterMotor2Temperatre;

    private final StatusSignal<Angle> turretPosition;
    private final StatusSignal<AngularVelocity> turretVelocity;
    private final StatusSignal<Voltage> turretVoltage;
    private final StatusSignal<Current> turretCurrent;
    private final StatusSignal<Temperature> turretTemperatre;

    private final StatusSignal<Angle> hoodPosition;
    private final StatusSignal<AngularVelocity> hoodVelocity;
    private final StatusSignal<Voltage> hoodVoltage;
    private final StatusSignal<Current> hoodCurrent;
    private final StatusSignal<Temperature> hoodTemperatre;

    //Debouncers
    private final Debouncer motorConnectDebouncer = new Debouncer(.25);

    public ShooterIOReal(){

        //Shooter Motor 1
        shooterMotor1 = new TalonFX(shooterMotor1CanID);

        TalonFXConfiguration shooterConfiguration =
      new TalonFXConfiguration()
          .withAudio(
              new AudioConfigs()
                  .withAllowMusicDurDisable(enableBeeps)
                  .withBeepOnBoot(enableBeeps)
                  .withBeepOnConfig(enableBeeps))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(shooterMotor1StatorCurrentLimit)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(shooterMotor1MotorSupplyLimitHigh)
                  .withSupplyCurrentLowerLimit(shooterMotor1MotorSupplyLimitLow)
                  .withSupplyCurrentLowerTime(shooterMotor1SupplyCurrentLowerTime)
                  .withSupplyCurrentLimitEnable(true))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  .withSensorToMechanismRatio(shooterMotor1MotorReduction))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(shooterMotor1Inverted)
                  .withNeutralMode(shooterMotor1NeutralMode))
          .withSlot0(
              new Slot0Configs()
                  .withKP(shooterMotor1Kp)
                  .withKI(shooterMotor1Ki)
                  .withKD(shooterMotor1Kd)
                  .withKG(0)
                  .withKV(shooterMotor1Kv)
                  .withKS(shooterMotor1Ks)
                  .withKA(shooterMotor1Ka))
          .withTorqueCurrent(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(shooterMotor1StatorCurrentLimit)
                  .withPeakReverseTorqueCurrent(shooterMotor1StatorCurrentLimit.unaryMinus()));

        tryUntilOk(5, ()-> shooterMotor1.getConfigurator().apply(shooterConfiguration, 0.25));

        shooterMotor1Position = shooterMotor1.getPosition();
        shooterMotor1Velocity = shooterMotor1.getVelocity();
        shooterMotor1Voltage = shooterMotor1.getMotorVoltage();
        shooterMotor1Current = shooterMotor1.getStatorCurrent();
        shooterMotor1Temperatre = shooterMotor1.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, shooterMotor1Position, shooterMotor1Velocity, shooterMotor1Voltage, shooterMotor1Current, shooterMotor1Temperatre);
        ParentDevice.optimizeBusUtilizationForAll(shooterMotor1);

        voltageRequest.EnableFOC = true;


        //Shooter Motor 2
        shooterMotor2 = new TalonFX(shooterMotor2CanID);

        TalonFXConfiguration shooterMotor2Configuration =
      new TalonFXConfiguration()
          .withAudio(
              new AudioConfigs()
                  .withAllowMusicDurDisable(enableBeeps)
                  .withBeepOnBoot(enableBeeps)
                  .withBeepOnConfig(enableBeeps))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(shooterMotor2StatorCurrentLimit)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(shooterMotor2MotorSupplyLimitHigh)
                  .withSupplyCurrentLowerLimit(shooterMotor2MotorSupplyLimitLow)
                  .withSupplyCurrentLowerTime(shooterMotor2SupplyCurrentLowerTime)
                  .withSupplyCurrentLimitEnable(true))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  .withSensorToMechanismRatio(shooterMotor2MotorReduction))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(shooterMotor2Inverted)
                  .withNeutralMode(shooterMotor2NeutralMode))
          .withSlot0(
              new Slot0Configs()
                  .withKP(shooterMotor2Kp)
                  .withKI(shooterMotor2Ki)
                  .withKD(shooterMotor2Kd)
                  .withKG(0)
                  .withKV(shooterMotor2Kv)
                  .withKS(shooterMotor2Ks)
                  .withKA(shooterMotor2Ka))
          .withTorqueCurrent(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(shooterMotor2StatorCurrentLimit)
                  .withPeakReverseTorqueCurrent(shooterMotor2StatorCurrentLimit.unaryMinus()));

        tryUntilOk(5, ()-> shooterMotor2.getConfigurator().apply(shooterConfiguration, 0.25));

        shooterMotor2Position = shooterMotor2.getPosition();
        shooterMotor2Velocity = shooterMotor2.getVelocity();
        shooterMotor2Voltage = shooterMotor2.getMotorVoltage();
        shooterMotor2Current = shooterMotor2.getStatorCurrent();
        shooterMotor2Temperatre = shooterMotor2.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, shooterMotor2Position, shooterMotor2Velocity, shooterMotor2Voltage, shooterMotor2Current, shooterMotor2Temperatre);
        ParentDevice.optimizeBusUtilizationForAll(shooterMotor2);

        voltageRequest.EnableFOC = true;


        //Turret Motor
        turretMotor = new TalonFX(turretMotorCanID);

        TalonFXConfiguration turretConfiguration =
      new TalonFXConfiguration()
          .withAudio(
              new AudioConfigs()
                  .withAllowMusicDurDisable(enableBeeps)
                  .withBeepOnBoot(enableBeeps)
                  .withBeepOnConfig(enableBeeps))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(turretMotorStatorCurrentLimit)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(turretMotorMotorSupplyLimitHigh)
                  .withSupplyCurrentLowerLimit(turretMotorMotorSupplyLimitLow)
                  .withSupplyCurrentLowerTime(turretMotorSupplyCurrentLowerTime)
                  .withSupplyCurrentLimitEnable(true))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  .withSensorToMechanismRatio(turretMotorMotorReduction))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(turretMotorInverted)
                  .withNeutralMode(turretMotorNeutralMode))
          .withSlot0(
              new Slot0Configs()
                  .withKP(turretMotorKp)
                  .withKI(turretMotorKi)
                  .withKD(turretMotorKd)
                  .withKG(0)
                  .withKV(turretMotorKv)
                  .withKS(turretMotorKs)
                  .withKA(turretMotorKa))
          .withTorqueCurrent(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(turretMotorStatorCurrentLimit)
                  .withPeakReverseTorqueCurrent(turretMotorStatorCurrentLimit.unaryMinus()));

        tryUntilOk(5, ()-> turretMotor.getConfigurator().apply(shooterConfiguration, 0.25));

        turretPosition = turretMotor.getPosition();
        turretVelocity = turretMotor.getVelocity();
        turretVoltage = turretMotor.getMotorVoltage();
        turretCurrent = turretMotor.getStatorCurrent();
        turretTemperatre = turretMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, turretPosition, turretVelocity, turretVoltage, turretCurrent, turretTemperatre);
        ParentDevice.optimizeBusUtilizationForAll(turretMotor);

        voltageRequest.EnableFOC = true;


        //Hood Motor
        hoodMotor = new TalonFX(hoodMotorCanID);

        TalonFXConfiguration hoodConfiguration =
      new TalonFXConfiguration()
          .withAudio(
              new AudioConfigs()
                  .withAllowMusicDurDisable(enableBeeps)
                  .withBeepOnBoot(enableBeeps)
                  .withBeepOnConfig(enableBeeps))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(hoodStatorCurrentLimit)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(hoodMotorSupplyLimitHigh)
                  .withSupplyCurrentLowerLimit(hoodMotorSupplyLimitLow)
                  .withSupplyCurrentLowerTime(hoodSupplyCurrentLowerTime)
                  .withSupplyCurrentLimitEnable(true))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  .withSensorToMechanismRatio(hoodMotorReduction))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(hoodInverted)
                  .withNeutralMode(hoodNeutralMode))
          .withSlot0(
              new Slot0Configs()
                  .withKP(hoodKp)
                  .withKI(hoodKi)
                  .withKD(hoodKd)
                  .withKG(0)
                  .withKV(hoodKv)
                  .withKS(hoodKs)
                  .withKA(hoodKa))
          .withTorqueCurrent(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(hoodStatorCurrentLimit)
                  .withPeakReverseTorqueCurrent(hoodStatorCurrentLimit.unaryMinus()));

        tryUntilOk(5, ()-> hoodMotor.getConfigurator().apply(shooterConfiguration, 0.25));

        hoodPosition = hoodMotor.getPosition();
        hoodVelocity = hoodMotor.getVelocity();
        hoodVoltage = hoodMotor.getMotorVoltage();
        hoodCurrent = hoodMotor.getStatorCurrent();
        hoodTemperatre = hoodMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, hoodPosition, hoodVelocity, hoodVoltage, hoodCurrent, hoodTemperatre);
        ParentDevice.optimizeBusUtilizationForAll(hoodMotor);

        voltageRequest.EnableFOC = true;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        var shooterMotor1Status = BaseStatusSignal.refreshAll(shooterMotor1Position, shooterMotor1Velocity, shooterMotor1Voltage, shooterMotor1Current, shooterMotor1Temperatre);
        var shooterMotor2Status = BaseStatusSignal.refreshAll(shooterMotor2Position, shooterMotor2Velocity, shooterMotor2Voltage, shooterMotor2Current, shooterMotor2Temperatre);
        var turretMotorStatus = BaseStatusSignal.refreshAll(turretPosition, turretVelocity, turretVoltage, turretCurrent, turretTemperatre);
        var hoodMotorStatus = BaseStatusSignal.refreshAll(hoodPosition, hoodVelocity, hoodVoltage, hoodCurrent, hoodTemperatre);

        inputs.shooterMotor1Connected = motorConnectDebouncer.calculate(shooterMotor1Status.isOK());
        inputs.shooterMotor1Position = shooterMotor1Position.getValue();
        inputs.shooterMotor1Velocity = shooterMotor1Velocity.getValue();
        inputs.shooterMotor1AppliedVolts = shooterMotor1Voltage.getValue();
        inputs.shooterMotor1Current = shooterMotor1Current.getValue();
        inputs.shooterMotor1Temp = shooterMotor1Temperatre.getValue();
        
        inputs.shooterMotor2Connected = motorConnectDebouncer.calculate(shooterMotor2Status.isOK());
        inputs.shooterMotor2Position = shooterMotor2Position.getValue();
        inputs.shooterMotor2Velocity = shooterMotor2Velocity.getValue();
        inputs.shooterMotor2AppliedVolts = shooterMotor2Voltage.getValue();
        inputs.shooterMotor2Current = shooterMotor2Current.getValue();
        inputs.shooterMotor2Temp = shooterMotor2Temperatre.getValue();

        inputs.turretMotorConnected = motorConnectDebouncer.calculate(turretMotorStatus.isOK());
        inputs.turretMotorPosition = turretPosition.getValue();
        inputs.turretMotorVelocity = turretVelocity.getValue();
        inputs.turretMotorAppliedVolts = turretVoltage.getValue();
        inputs.turretMotorCurrent = turretCurrent.getValue();
        inputs.turretMotorTemp = turretTemperatre.getValue();

        inputs.hoodMotorConnected = motorConnectDebouncer.calculate(hoodMotorStatus.isOK());
        inputs.hoodMotorPosition = hoodPosition.getValue();
        inputs.hoodMotorVelocity = hoodVelocity.getValue();
        inputs.hoodMotorAppliedVolts = hoodVoltage.getValue();
        inputs.hoodMotorCurrent = hoodCurrent.getValue();
        inputs.hoodMotorTemp = hoodTemperatre.getValue();
    }

    @Override
    public void setShooter(Voltage volts) {
        shooterMotor1.setControl(voltageRequest.withOutput(volts));
        shooterMotor2.setControl(voltageRequest.withOutput(volts));
        turretMotor.setControl(voltageRequest.withOutput(volts));
        hoodMotor.setControl(voltageRequest.withOutput(volts));
    }
}

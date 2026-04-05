package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.lib.util.PhoenixUtil.tryUntilOk;
import static frc.robot.lib.util.SparkUtil.*;
import static frc.robot.subsystems.hopper.HopperConstants.*;

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
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.lib.util.SparkUtil;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class HopperIOReal implements HopperIO {
  // Motors
  public final TalonFX bottomHopperMotor;
  public final SparkMax hopperTopMotor;

  public final RelativeEncoder hopperTopMotorEncoder;

  public final AbsoluteEncoder turretAbsoluteEncoder;

  // Conrtol Requests
  private final VoltageOut voltageRequest = new VoltageOut(0);

  // Status Signals
  private final StatusSignal<Angle> hopperPosition;
  private final StatusSignal<AngularVelocity> hopperVelocity;
  private final StatusSignal<Voltage> hopperVoltage;
  private final StatusSignal<Current> hopperStatorCurrent;
  private final StatusSignal<Temperature> hopperTemperature;
  private final StatusSignal<Current> hopperSupplyCurrent;

  // Debouncers
  private final Debouncer topMotorConnectDebouncer = new Debouncer(.25);
  private final Debouncer bottomMotorConnectDebouncer = new Debouncer(.25);

  public HopperIOReal() {
    bottomHopperMotor = new TalonFX(hopperMotorCanID);

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

    tryUntilOk(5, () -> bottomHopperMotor.getConfigurator().apply(hopperConfiguration, 0.25));

    hopperPosition = bottomHopperMotor.getPosition();
    hopperVelocity = bottomHopperMotor.getVelocity();
    hopperVoltage = bottomHopperMotor.getMotorVoltage();
    hopperStatorCurrent = bottomHopperMotor.getStatorCurrent();
    hopperTemperature = bottomHopperMotor.getDeviceTemp();
    hopperSupplyCurrent = bottomHopperMotor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        hopperPosition,
        hopperVelocity,
        hopperVoltage,
        hopperStatorCurrent,
        hopperTemperature,
        hopperSupplyCurrent);
    ParentDevice.optimizeBusUtilizationForAll(bottomHopperMotor);

    voltageRequest.EnableFOC = true;

    hopperTopMotor = new SparkMax(topRollerCanID, MotorType.kBrushless);

    SparkMaxConfig topMotorConfig = new SparkMaxConfig();
    topMotorConfig.idleMode(topRollerIdleMode);
    topMotorConfig.smartCurrentLimit(topRollerCurrentLimit);
    topMotorConfig.inverted(topRollerInverted);
    topMotorConfig.encoder.positionConversionFactor(topRollerRatio);
    topMotorConfig.encoder.velocityConversionFactor(topRollerRatio);
    topMotorConfig.absoluteEncoder.inverted(true);
    topMotorConfig.absoluteEncoder.positionConversionFactor(1.0);

    SparkUtil.tryUntilOk(
        hopperTopMotor,
        5,
        () ->
            hopperTopMotor.configure(
                topMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    hopperTopMotorEncoder = hopperTopMotor.getEncoder();

    turretAbsoluteEncoder = hopperTopMotor.getAbsoluteEncoder();
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    var bottomMotorStatus =
        BaseStatusSignal.refreshAll(
            hopperPosition,
            hopperVelocity,
            hopperVoltage,
            hopperStatorCurrent,
            hopperTemperature,
            hopperSupplyCurrent);

    inputs.bottomMotorConnected = bottomMotorConnectDebouncer.calculate(bottomMotorStatus.isOK());
    inputs.bottomMotorPosition = hopperPosition.getValue();
    inputs.bottomMotorVelocity = hopperVelocity.getValue();
    inputs.bottomMotorAppliedVolts = hopperVoltage.getValue();
    inputs.bottomMotorStatorCurrent = hopperStatorCurrent.getValue();
    inputs.bottomMotorTemp = hopperTemperature.getValue();
    inputs.bottomMotorSupplyCurrent = hopperSupplyCurrent.getValue();

    sparkStickyFault = false;
    ifOk(
        hopperTopMotor,
        hopperTopMotor::getMotorTemperature,
        (value) -> inputs.topMotorTemp = Celsius.of(value));
    ifOk(
        hopperTopMotor,
        new DoubleSupplier[] {hopperTopMotor::getAppliedOutput, hopperTopMotor::getBusVoltage},
        (value) -> inputs.topMotorAppliedVolts = Volts.of(value[0] * value[1]));
    ifOk(
        hopperTopMotor,
        hopperTopMotor::getOutputCurrent,
        (value) -> inputs.topMotorStatorCurrent = Amps.of(value));
    ifOk(
        hopperTopMotor,
        hopperTopMotorEncoder::getPosition,
        (value) -> inputs.topMotorPosition = Rotations.of(value));
    ifOk(
        hopperTopMotor,
        hopperTopMotorEncoder::getVelocity,
        (value) -> inputs.topMotorVelocity = RotationsPerSecond.of(value * 60));
    inputs.topMotorConnected = topMotorConnectDebouncer.calculate(!sparkStickyFault);

    Logger.recordOutput("Turret Rotation", Rotations.of(turretAbsoluteEncoder.getPosition()));
  }

  @Override
  public void setVoltage(Voltage volts) {
    bottomHopperMotor.setControl(voltageRequest.withOutput(volts));
    hopperTopMotor.setVoltage(volts);
  }

  @Override
  public AbsoluteEncoder getTurretAbsoluteEncoder() {
    return turretAbsoluteEncoder;
  }
}

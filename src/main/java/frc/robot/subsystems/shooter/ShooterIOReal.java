package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.lib.util.PhoenixUtil.tryUntilOk;
import static frc.robot.lib.util.SparkUtil.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.lib.util.SparkUtil;
import java.util.function.DoubleSupplier;

public class ShooterIOReal implements ShooterIO {
  // Motors
  private final TalonFX shooterMotor1;
  private final TalonFX shooterMotor2;
  private final TalonFX turretMotor;
  private final SparkMax hoodMotor;

  // Encoder
  private final DutyCycleEncoder turretEncoder;

  // Conrtol Requests
  private final VoltageOut shooterVoltageRequest = new VoltageOut(0);
  private final VelocityVoltage shooterVelocityRequest = new VelocityVoltage(0);

  private final VoltageOut shooter2VoltageRequest = new VoltageOut(0);
  private final VelocityVoltage shooter2VelocityRequest = new VelocityVoltage(0);

  private final MotionMagicVoltage turretMotionMagic = new MotionMagicVoltage(0);
  private final VoltageOut turretvoltageRequest = new VoltageOut(0);

  // Status Signals
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

  private final SparkClosedLoopController hoodPIDController;
  private final RelativeEncoder hoodEncoder;

  // Debouncers
  private final Debouncer shooterMotor1ConnectedDebouncer = new Debouncer(.25);
  private final Debouncer shooterMotor2ConnectedDebouncer = new Debouncer(.25);
  private final Debouncer turretMotorConnectedDebouncer = new Debouncer(.25);
  private final Debouncer hoodMotorConnectedDebouncer = new Debouncer(.25);
  private final Debouncer turretAbsoluteEncoderDebouncer = new Debouncer(.25);
  private boolean turretInitialized = false;

  public ShooterIOReal() {

    turretEncoder = new DutyCycleEncoder(TurretEncoderPin, 1, 0.6316);

    // Shooter Motor 1
    shooterMotor1 = new TalonFX(shooterMotor1CanID);

    TalonFXConfiguration shooterMotor1Configuration =
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
                    .withKP(shooterKp)
                    .withKI(shooterKi)
                    .withKD(shooterKd)
                    .withKG(0)
                    .withKV(shooterKv)
                    .withKS(shooterKs)
                    .withKA(shooterKa))
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(shooterMotor1StatorCurrentLimit)
                    .withPeakReverseTorqueCurrent(shooterMotor1StatorCurrentLimit.unaryMinus()));

    tryUntilOk(5, () -> shooterMotor1.getConfigurator().apply(shooterMotor1Configuration, 0.25));

    shooterMotor1Position = shooterMotor1.getPosition();
    shooterMotor1Velocity = shooterMotor1.getVelocity();
    shooterMotor1Voltage = shooterMotor1.getMotorVoltage();
    shooterMotor1Current = shooterMotor1.getStatorCurrent();
    shooterMotor1Temperatre = shooterMotor1.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        shooterMotor1Position,
        shooterMotor1Velocity,
        shooterMotor1Voltage,
        shooterMotor1Current,
        shooterMotor1Temperatre);
    ParentDevice.optimizeBusUtilizationForAll(shooterMotor1);

    // Disable FOC on all motors due to weird issues being reported
    shooterVoltageRequest.EnableFOC = false;
    shooterVelocityRequest.EnableFOC = false;

    // Shooter Motor 2
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
                    .withKP(shooterKp)
                    .withKI(shooterKi)
                    .withKD(shooterKd)
                    .withKG(0)
                    .withKV(shooterKv)
                    .withKS(shooterKs)
                    .withKA(shooterKa))
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(shooterMotor2StatorCurrentLimit)
                    .withPeakReverseTorqueCurrent(shooterMotor2StatorCurrentLimit.unaryMinus()));

    tryUntilOk(5, () -> shooterMotor2.getConfigurator().apply(shooterMotor2Configuration, 0.25));

    shooterMotor2Position = shooterMotor2.getPosition();
    shooterMotor2Velocity = shooterMotor2.getVelocity();
    shooterMotor2Voltage = shooterMotor2.getMotorVoltage();
    shooterMotor2Current = shooterMotor2.getStatorCurrent();
    shooterMotor2Temperatre = shooterMotor2.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        shooterMotor2Position,
        shooterMotor2Velocity,
        shooterMotor2Voltage,
        shooterMotor2Current,
        shooterMotor2Temperatre);
    ParentDevice.optimizeBusUtilizationForAll(shooterMotor2);

    // Disable FOC on all motors due to weird issues being reported
    shooter2VoltageRequest.EnableFOC = false;
    shooter2VelocityRequest.EnableFOC = false;

    // Turret Motor
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
                    .withPeakReverseTorqueCurrent(turretMotorStatorCurrentLimit.unaryMinus()))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(turretcruiseVelocity)
                    .withMotionMagicAcceleration(turretAcceleration));

    tryUntilOk(5, () -> turretMotor.getConfigurator().apply(turretConfiguration, 0.25));

    turretMotor.setPosition(Rotations.of(turretEncoder.get()));

    turretPosition = turretMotor.getPosition();
    turretVelocity = turretMotor.getVelocity();
    turretVoltage = turretMotor.getMotorVoltage();
    turretCurrent = turretMotor.getStatorCurrent();
    turretTemperatre = turretMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, turretPosition, turretVelocity, turretVoltage, turretCurrent, turretTemperatre);
    ParentDevice.optimizeBusUtilizationForAll(turretMotor);

    // Disable FOC on all motors due to weird issues being reported
    turretvoltageRequest.EnableFOC = false;
    turretMotionMagic.EnableFOC = false;

    // Hood Motor
    hoodMotor = new SparkMax(hoodMotorCanID, MotorType.kBrushless);

    SparkMaxConfig hoodMotorConfig = new SparkMaxConfig();

    hoodMotorConfig.idleMode(hoodNeutralMode);
    hoodMotorConfig.inverted(hoodInverted);
    hoodMotorConfig.smartCurrentLimit(hoodStatorCurrentLimit);
    hoodMotorConfig.encoder.positionConversionFactor(hoodMotorReduction);
    hoodMotorConfig.encoder.velocityConversionFactor(hoodMotorReduction);
    hoodMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(hoodKp)
        .i(hoodKi)
        .d(hoodKd)
        .outputRange(-1, 1)
        .feedForward
        .kS(hoodKs);

    SparkUtil.tryUntilOk(
        hoodMotor,
        5,
        () -> {
          return hoodMotor.configure(
              hoodMotorConfig,
              com.revrobotics.ResetMode.kResetSafeParameters,
              com.revrobotics.PersistMode.kPersistParameters);
        });

    hoodPIDController = hoodMotor.getClosedLoopController();
    hoodEncoder = hoodMotor.getEncoder();

    hoodEncoder.setPosition(0);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    var shooterMotor1Status =
        BaseStatusSignal.refreshAll(
            shooterMotor1Position,
            shooterMotor1Velocity,
            shooterMotor1Voltage,
            shooterMotor1Current,
            shooterMotor1Temperatre);
    var shooterMotor2Status =
        BaseStatusSignal.refreshAll(
            shooterMotor2Position,
            shooterMotor2Velocity,
            shooterMotor2Voltage,
            shooterMotor2Current,
            shooterMotor2Temperatre);
    var turretMotorStatus =
        BaseStatusSignal.refreshAll(
            turretPosition, turretVelocity, turretVoltage, turretCurrent, turretTemperatre);

    inputs.shooterMotor1Connected =
        shooterMotor1ConnectedDebouncer.calculate(shooterMotor1Status.isOK());
    inputs.shooterMotor1Position = shooterMotor1Position.getValue();
    inputs.shooterMotor1Velocity = shooterMotor1Velocity.getValue();
    inputs.shooterMotor1AppliedVolts = shooterMotor1Voltage.getValue();
    inputs.shooterMotor1Current = shooterMotor1Current.getValue();
    inputs.shooterMotor1Temp = shooterMotor1Temperatre.getValue();

    inputs.shooterMotor2Connected =
        shooterMotor2ConnectedDebouncer.calculate(shooterMotor2Status.isOK());
    inputs.shooterMotor2Position = shooterMotor2Position.getValue();
    inputs.shooterMotor2Velocity = shooterMotor2Velocity.getValue();
    inputs.shooterMotor2AppliedVolts = shooterMotor2Voltage.getValue();
    inputs.shooterMotor2Current = shooterMotor2Current.getValue();
    inputs.shooterMotor2Temp = shooterMotor2Temperatre.getValue();

    inputs.turretMotorConnected = turretMotorConnectedDebouncer.calculate(turretMotorStatus.isOK());
    inputs.turretMotorPosition = turretPosition.getValue();
    inputs.turretMotorVelocity = turretVelocity.getValue();
    inputs.turretMotorAppliedVolts = turretVoltage.getValue();
    inputs.turretMotorCurrent = turretCurrent.getValue();
    inputs.turretMotorTemp = turretTemperatre.getValue();

    sparkStickyFault = false;
    ifOk(
        hoodMotor,
        hoodEncoder::getPosition,
        (value) -> inputs.hoodMotorPosition = Rotations.of(value));
    ifOk(
        hoodMotor,
        hoodEncoder::getVelocity,
        (value) -> inputs.hoodMotorVelocity = RotationsPerSecond.of(value * 60));
    ifOk(
        hoodMotor,
        new DoubleSupplier[] {hoodMotor::getAppliedOutput, hoodMotor::getBusVoltage},
        (value) -> inputs.hoodMotorAppliedVolts = Volts.of(value[0] * value[1]));
    ifOk(
        hoodMotor,
        hoodMotor::getOutputCurrent,
        (value) -> inputs.hoodMotorCurrent = Amps.of(value));
    ifOk(
        hoodMotor,
        hoodMotor::getMotorTemperature,
        (value) -> inputs.hoodMotorTemp = Celsius.of(value));
    inputs.hoodMotorConnected = hoodMotorConnectedDebouncer.calculate(!sparkStickyFault);

    inputs.turretAbsoluteEncoderConnected =
        turretAbsoluteEncoderDebouncer.calculate(turretEncoder.isConnected());
    inputs.turretAbsoluteAngle = Rotations.of(turretEncoder.get());

    if (!turretInitialized) {
      turretMotor.setPosition(Rotations.of(turretEncoder.get()));

      if (turretPosition.getValue().isNear(Rotations.of(turretEncoder.get()), Degrees.of(.5))) {
        turretInitialized = true;
      }
    }
  }

  @Override
  public void setShooter(Voltage volts) {
    shooterMotor1.setControl(shooterVoltageRequest.withOutput(volts));
    shooterMotor2.setControl(shooterVoltageRequest.withOutput(volts));
  }

  @Override
  public void setShooter(AngularVelocity velocity) {
    shooterMotor1.setControl(shooterVelocityRequest.withVelocity(velocity));
    shooterMotor2.setControl(shooterVelocityRequest.withVelocity(velocity));
  }

  @Override
  public void setTurret(Voltage volts) {
    turretMotor.setControl(turretvoltageRequest.withOutput(volts));
  }

  @Override
  public void setTurret(Angle angle) {
    turretMotor.setControl(turretMotionMagic.withPosition(angle));
  }

  //   @Override
  //   public void setPID(double kp, double ki, double kd, double ks, double kv, double ka) {
  //     Slot0Configs newConfig =
  //         new Slot0Configs().withKP(kp).withKI(ki).withKD(kd).withKS(ks).withKV(kv).withKA(ka);
  //     shooterMotor1.getConfigurator().apply(newConfig);
  //     shooterMotor2.getConfigurator().apply(newConfig);
  //   }

  @Override
  public void setHood(Voltage volts) {
    hoodMotor.setVoltage(volts);
  }

  @Override
  public void setHood(Angle angle) {
    hoodPIDController.setSetpoint(angle.in(Rotations), ControlType.kPosition);
  }
}

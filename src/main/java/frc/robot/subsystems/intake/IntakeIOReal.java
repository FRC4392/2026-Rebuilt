package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.lib.util.PhoenixUtil.tryUntilOk;
import static frc.robot.subsystems.intake.IntakeConstants.*;

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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOReal implements IntakeIO {

  // Motors
  private final TalonFX extensionMotor;
  private final TalonFX leftRollerMotor;
  private final TalonFX rightRollerMotor;

  // Control Requests
  private final VoltageOut extensionVoltageRequest = new VoltageOut(0);
  private final MotionMagicVoltage extensionPositionRequest = new MotionMagicVoltage(0);

  private final VoltageOut rollerVoltageRequest = new VoltageOut(0);
  private final TorqueCurrentFOC rollTorqueCurrentRequest = new TorqueCurrentFOC(0);

  // Status Signals
  private final StatusSignal<Angle> extensionPosition;
  private final StatusSignal<AngularVelocity> extensionVelocity;
  private final StatusSignal<Voltage> extensionVoltage;
  private final StatusSignal<Current> extensionCurrent;
  private final StatusSignal<Temperature> extensionTemperature;

  private final StatusSignal<Angle> leftRollerPosition;
  private final StatusSignal<AngularVelocity> leftRollerVelocity;
  private final StatusSignal<Voltage> leftRollerVoltage;
  private final StatusSignal<Current> leftRollerCurrent;
  private final StatusSignal<Temperature> leftRollerTemperature;

  private final StatusSignal<Angle> rightRollerPosition;
  private final StatusSignal<AngularVelocity> rightRollerVelocity;
  private final StatusSignal<Voltage> rightRollerVoltage;
  private final StatusSignal<Current> rightRollerCurrent;
  private final StatusSignal<Temperature> rightRollerTemperature;

  // Debouncers
  private final Debouncer extensionMotorConnectedDebouncer = new Debouncer(.25);
  private final Debouncer leftRollerMotorConnectedDebouncer = new Debouncer(.25);
  private final Debouncer rightRollerMotorConnectedDebouncer = new Debouncer(.25);

  public IntakeIOReal() {
    // Extension Configuration
    extensionMotor = new TalonFX(extensionCanID);

    TalonFXConfiguration extensionConfiguration =
        new TalonFXConfiguration()
            .withAudio(
                new AudioConfigs()
                    .withAllowMusicDurDisable(enableBeeps)
                    .withBeepOnBoot(enableBeeps)
                    .withBeepOnConfig(enableBeeps))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(extenstionStatorCurrentLimit)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(extensionMotorSupplyLimitHigh)
                    .withSupplyCurrentLowerLimit(extensionMotorSupplyLimitLow)
                    .withSupplyCurrentLowerTime(extensionSupplyCurrentLowerTime)
                    .withSupplyCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(extensionMotorReduction))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(extensionInverted)
                    .withNeutralMode(extensionNeutralMode))
            .withSlot0(
                new Slot0Configs()
                    .withKP(extensionKp)
                    .withKI(extensionKi)
                    .withKD(extensionKd)
                    .withKG(0)
                    .withKV(extensionKv)
                    .withKS(extensionKs)
                    .withKA(extensionKa))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicAcceleration(extensionMotorMaxAcceleration)
                    .withMotionMagicCruiseVelocity(extensionMotorMaxSpeed))
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(extenstionStatorCurrentLimit)
                    .withPeakReverseTorqueCurrent(extenstionStatorCurrentLimit.unaryMinus()));

    tryUntilOk(5, () -> extensionMotor.getConfigurator().apply(extensionConfiguration, 0.25));

    extensionPosition = extensionMotor.getPosition();
    extensionVelocity = extensionMotor.getVelocity();
    extensionVoltage = extensionMotor.getMotorVoltage();
    extensionCurrent = extensionMotor.getStatorCurrent();
    extensionTemperature = extensionMotor.getDeviceTemp();

    extensionVoltageRequest.EnableFOC = true;
    extensionPositionRequest.EnableFOC = true;
    extensionPositionRequest.Slot = 0;

    // Roller Configuration
    leftRollerMotor = new TalonFX(rightRollerCanID);
    rightRollerMotor = new TalonFX(rightRollerCanID);

    TalonFXConfiguration leftRollerConfiguration =
        new TalonFXConfiguration()
            .withAudio(
                new AudioConfigs()
                    .withAllowMusicDurDisable(enableBeeps)
                    .withBeepOnBoot(enableBeeps)
                    .withBeepOnConfig(enableBeeps))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(rollerStatorCurrentLimit)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(rollerMotorSupplyLimitHigh)
                    .withSupplyCurrentLowerLimit(rollerMotorSupplyLimitLow)
                    .withSupplyCurrentLowerTime(rollerSupplyCurrentLowerTime)
                    .withSupplyCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(rollerMotorReduction))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(leftRollerInverted)
                    .withNeutralMode(rollerNeutralMode))
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(rollerStatorCurrentLimit)
                    .withPeakReverseTorqueCurrent(rollerStatorCurrentLimit.unaryMinus()));

    TalonFXConfiguration rightRollerConfiguration = leftRollerConfiguration;
    rightRollerConfiguration.MotorOutput.Inverted = rightRollerInverted;

    tryUntilOk(5, () -> leftRollerMotor.getConfigurator().apply(leftRollerConfiguration));
    tryUntilOk(5, () -> rightRollerMotor.getConfigurator().apply(rightRollerConfiguration));

    leftRollerPosition = leftRollerMotor.getPosition();
    leftRollerVelocity = leftRollerMotor.getVelocity();
    leftRollerVoltage = leftRollerMotor.getMotorVoltage();
    leftRollerCurrent = leftRollerMotor.getStatorCurrent();
    leftRollerTemperature = leftRollerMotor.getDeviceTemp();

    rightRollerPosition = rightRollerMotor.getPosition();
    rightRollerVelocity = rightRollerMotor.getVelocity();
    rightRollerVoltage = rightRollerMotor.getMotorVoltage();
    rightRollerCurrent = rightRollerMotor.getStatorCurrent();
    rightRollerTemperature = rightRollerMotor.getDeviceTemp();

    rollerVoltageRequest.EnableFOC = true;

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        extensionPosition,
        extensionVelocity,
        extensionVoltage,
        extensionCurrent,
        extensionTemperature,
        leftRollerPosition,
        leftRollerVelocity,
        leftRollerVoltage,
        leftRollerCurrent,
        leftRollerTemperature,
        rightRollerPosition,
        rightRollerVelocity,
        rightRollerVoltage,
        rightRollerCurrent,
        rightRollerTemperature);
    ParentDevice.optimizeBusUtilizationForAll(extensionMotor, leftRollerMotor, rightRollerMotor);

    tryUntilOk(5, () -> rightRollerMotor.setControl(new Follower(leftRollerCanID, MotorAlignmentValue.Opposed)));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var extensionStatus =
        BaseStatusSignal.refreshAll(
            extensionPosition,
            extensionVelocity,
            extensionVoltage,
            extensionCurrent,
            extensionTemperature);

    inputs.extensionMotorConnected =
        extensionMotorConnectedDebouncer.calculate(extensionStatus.isOK());
    inputs.extensionMotorPosition = extensionPosition.getValue();
    inputs.extensionMotorVelocity = extensionVelocity.getValue();
    inputs.extensionMotorAppliedVolts = extensionVoltage.getValue();
    inputs.extensionMotorCurrent = extensionCurrent.getValue();
    inputs.extensionMotorTemp = extensionTemperature.getValue();

    var leftRollerStatus =
        BaseStatusSignal.refreshAll(
            leftRollerPosition,
            leftRollerVelocity,
            leftRollerVoltage,
            leftRollerCurrent,
            leftRollerTemperature);

    inputs.leftRollerMotorConnected =
        leftRollerMotorConnectedDebouncer.calculate(leftRollerStatus.isOK());
    inputs.leftRollerMotorPosition = leftRollerPosition.getValue();
    inputs.leftRollerMotorVelocity = leftRollerVelocity.getValue();
    inputs.leftRollerMotorAppliedVolts = leftRollerVoltage.getValue();
    inputs.leftRollerMotorCurrent = leftRollerCurrent.getValue();
    inputs.leftRollerMotorTemp = leftRollerTemperature.getValue();

    var rightRollerStatus =
        BaseStatusSignal.refreshAll(
            rightRollerPosition,
            rightRollerVelocity,
            rightRollerVoltage,
            rightRollerCurrent,
            rightRollerTemperature);

    inputs.rightRollerMotorConnected =
        rightRollerMotorConnectedDebouncer.calculate(rightRollerStatus.isOK());
    inputs.rightRollerMotorPosition = rightRollerPosition.getValue();
    inputs.rightRollerMotorVelocity = rightRollerVelocity.getValue();
    inputs.rightRollerMotorAppliedVolts = rightRollerVoltage.getValue();
    inputs.rightRollerMotorCurrent = rightRollerCurrent.getValue();
    inputs.rightRollerMotorTemp = rightRollerTemperature.getValue();
  }

  @Override
  public void setExtension(Voltage volts) {
    extensionMotor.setControl(extensionVoltageRequest.withOutput(volts));
  }

  @Override
  public void setExtension(Distance distance) {
    Angle position = Radians.of(distance.in(Meters) / (extensionDriveDiameter.in(Meters) / 2.0));
    extensionMotor.setControl(extensionPositionRequest.withPosition(position));
  }

  @Override
  public void setRoller(Voltage volts) {
    leftRollerMotor.setControl(rollerVoltageRequest.withOutput(volts));
    //rightRollerMotor.setControl(new Follower(leftRollerCanID, MotorAlignmentValue.Opposed));
  }

  @Override
  public void setRoller(Current current) {
    leftRollerMotor.setControl(rollTorqueCurrentRequest.withOutput(current));
    //rightRollerMotor.setControl(new Follower(leftRollerCanID, MotorAlignmentValue.Opposed));
  }
}

package frc.robot.operatorinterface;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.operatorinterface.OperatorInterfaceConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.DeceiverRobotState;
import frc.robot.subsystems.swerve.SwerveControlSignal;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class OperatorInterface extends SubsystemBase {
  // Robot state instance
  private final DeceiverRobotState robotState;

  // Controllers
  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;

  // Dashboard
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedNetworkBoolean resetRobotStateBoolean;

  // Alerts
  private final Alert autoAlert = new Alert("Select an autonomous mode! 😟", AlertType.kError);
  private final Alert driverControllerAlert =
      new Alert("Driver Controller Disconnected 🎮", AlertType.kError);
  private final Alert operatorControllerAlert =
      new Alert("Operator Controller Disconnected 🎮", AlertType.kError);

  /**
   * Constructor
   *
   * @param state robot state instance
   */
  public OperatorInterface(DeceiverRobotState state) {
    robotState = state;

    // Set up controllers
    driverController = new CommandXboxController(DriverControllerPort);
    operatorController = new CommandXboxController(OperatorControllerPort);

    // Setup reset robot state button
    resetRobotStateBoolean = new LoggedNetworkBoolean("resetRobotState");
    resetRobotStateBoolean.setDefault(false);

    // Set up auto chooser
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addDefaultOption("None", noAuto);

    // Remove controller disconnected message, we handle this on our own
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /** Update dashboard alerts */
  private void updateAlerts() {
    // Set controller connected alerts
    driverControllerAlert.set(!driverController.isConnected() && RobotBase.isReal());
    operatorControllerAlert.set(!operatorController.isConnected() && RobotBase.isReal());

    // Check that an auto has been selected
    autoAlert.set(!robotState.getWasAuto() && autoChooser.get() == noAuto);
  }

  @Override
  public void periodic() {
    updateAlerts();

    // Send match time to the Dashboard
    SmartDashboard.putNumber("MatchTime", DriverStation.getMatchTime());

    // Reset robot state if button is pressed
    if (resetRobotStateBoolean.get() && robotState.isDisabled() && !DriverStation.isFMSAttached()) {
      robotState.resetState();
      resetRobotStateBoolean.set(false);
    }
  }

  /**
   * Get the currently selected auto command
   *
   * @return The currently selected auto command
   */
  public Command getAutoCommand() {
    return autoChooser.get();
  }

  /**
   * Add an new auto option to the dashboard
   *
   * @param name Name of the command
   * @param autoCommand The Command to run when selected
   */
  public void addAutoOption(String name, Command autoCommand) {
    autoChooser.addOption(name, autoCommand);
  }

  /**
   * Starts the joystick rumbling and then stops it at the completion of the command.
   *
   * @return Command to rumble the joystick
   */
  public Command joystickRumbleCommand() {
    return this.startEnd(
        () -> {
          driverController.setRumble(RumbleType.kBothRumble, 1.0);
          operatorController.setRumble(RumbleType.kBothRumble, 1.0);
        },
        () -> {
          driverController.setRumble(RumbleType.kBothRumble, 0.0);
          operatorController.setRumble(RumbleType.kBothRumble, 0.0);
        });
  }

  // Serve controls

  /**
   * Get trigger to stop with wheels in an x shape
   *
   * @return Trigger to activate stopping with x
   */
  public Trigger stopWithXTrigger() {
    return driverController.x();
  }

  /**
   * Get trigger to stop with wheels in an x shape
   *
   * @return Trigger to activate stopping with x
   */
  public Trigger restGyroTrigger() {
    return driverController.start();
  }

  /**
   * Get SwerveControlSignal that represents all data need to drive swerve
   *
   * @return
   */
  public SwerveControlSignal getSwerveControlSignal() {
    return new SwerveControlSignal(
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> -driverController.getRightX(),
        () -> driverController.getHID().getRightStickButton());
  }

  public record SwerveAngleControlSignal(
      DoubleSupplier x, DoubleSupplier y, Supplier<Rotation2d> angle, BooleanSupplier fastMode) {}
  ;

  public SwerveAngleControlSignal getSwerveAngleControlSignal() {
    return new SwerveAngleControlSignal(
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> new Rotation2d(driverController.getRightY(), driverController.getRightX()),
        () -> driverController.getHID().getRightStickButton());
  }

  public Trigger swerveAltMode() {
    return driverController.leftStick();
  }

  // Game Controls (vary by year)

  public Trigger hopperButton() {
    return operatorController.start();
  }

  public Trigger climberButton() {
    return operatorController.rightStick();
  }

  public Trigger indexerButton() {
    return operatorController.start();
  }

  public Trigger intakeButton() {
    return operatorController.y().or(driverController.leftTrigger());
  }

  // public Trigger outtakeButton() {
  //   return operatorController.rightBumper();
  // }

  public Trigger extendButton() {
    return operatorController.a();
  }

  public Trigger retractButton() {
    return operatorController.b();
  }

  public Trigger shooterButton() {
    return operatorController.rightTrigger();
  }

  public Supplier<Voltage> turretSpeedSupplier() {
    return () -> {
      return Volts.of(
          (operatorController.getRightTriggerAxis() - operatorController.getLeftTriggerAxis()) * 2);
    };
  }

  public Trigger testLeftTurret() {
    return operatorController.povLeft();
  }

  public Trigger testRightTurret() {
    return operatorController.povRight();
  }

  public Trigger testUpTurret() {
    return operatorController.povUp();
  }

  public Trigger testDownTurret() {
    return operatorController.povDown();
  }

  public Trigger feedStop() {
    return operatorController.leftBumper();
  }

  // Force Triggers
  public Trigger forceHub() {
    return operatorController.rightBumper();
  }

  public Trigger forceFeedLeft() {
    return operatorController.leftTrigger(0.1);
  }

  public Trigger forceFeedRight() {
    return operatorController.rightTrigger(0.1);
  }

  public Trigger trenchMode() {
    return operatorController.x().or(driverController.leftBumper());
  }

  public Trigger shiftOverride() {
    return operatorController.back();
  }

  public Trigger feedMode() {
    return operatorController.povDown();
  }

  public Supplier<Rotation2d> rotationSupplier() {
    return () -> {
      Rotation2d rotation =
          new Rotation2d(operatorController.getRightY(), operatorController.getRightX());

      return rotation;
    };
  }

  public BooleanSupplier operatorIsAngle() {
    return () -> {
      Translation2d distance =
          new Translation2d(driverController.getRightX(), driverController.getRightY());
      return distance.getNorm() > 0.1;
    };
  }

  public Trigger forceShoot() {
    return operatorController.start();
  }

  public Trigger demoLobMode() {
    return operatorController.rightBumper();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.geometry.AllianceFlipUtil;
import frc.robot.lib.geometry.Bounds;
import frc.robot.operatorinterface.OperatorInterface;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOReal;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOReal;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.TargetLocation;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveModuleIODeceivers;
import frc.robot.subsystems.swerve.SwerveModuleIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

public class RobotContainer {
  private final DeceiverRobotState robotState;

  // Subsystems
  public final Swerve swerve;

  public final Shooter shooter;
  public final Indexer indexer;
  public final Hopper hopper;
  // public final Climber climber;
  public final Intake intake;
  public final Vision vision;

  // Operator Interface
  private final OperatorInterface operatorInterface;

  /**
   * Constructor
   *
   * @param state RobotState object to track the state of the robot
   */
  public RobotContainer() {
    robotState = DeceiverRobotState.getInstance();

    // Lower brownout voltage
    RobotController.setBrownoutVoltage(6.0);

    // Create subsystem hardware
    switch (RobotConstants.currentMode) {
      case COMMISIONING:
        // Fall Through
      case REAL:
        // Real Robot, use real hardware interfaces
        swerve =
            new Swerve(
                new GyroIOPigeon2(),
                new SwerveModuleIODeceivers(0),
                new SwerveModuleIODeceivers(1),
                new SwerveModuleIODeceivers(2),
                new SwerveModuleIODeceivers(3));

        shooter = new Shooter(new ShooterIOReal());
        indexer = new Indexer(new IndexerIOReal());
        // climber = new Climber(new ClimberIOReal());
        hopper = new Hopper(new HopperIOReal());
        intake = new Intake(new IntakeIOReal());
        vision =
            new Vision(
                swerve::addVisionMeasurement,
                new VisionIOLimelight("limelight-one", swerve::getRotation),
                new VisionIOLimelight("limelight-two", swerve::getRotation),
                new VisionIOLimelight("limelight-three", swerve::getRotation),
                new VisionIOLimelight("limelight-four", swerve::getRotation));
        break;
      case SIM:
        // Simulated robot use simulation hardware interfaces
        swerve =
            new Swerve(
                new GyroIO() {},
                new SwerveModuleIOSim(),
                new SwerveModuleIOSim(),
                new SwerveModuleIOSim(),
                new SwerveModuleIOSim());

        shooter = new Shooter(new ShooterIOSim());
        indexer = new Indexer(new IndexerIOSim());
        // climber = new Climber(new ClimberIOSim());
        hopper = new Hopper(new HopperIOSim());
        intake = new Intake(new IntakeIOSim());
        vision =
            new Vision(
                swerve::addVisionMeasurement,
                new VisionIOPhotonVisionSim("Camera1", new Transform3d(), swerve::getPose));
        break;
      default:
        // Replay, don't use hardware
        swerve =
            new Swerve(
                new GyroIO() {},
                new SwerveModuleIO() {},
                new SwerveModuleIO() {},
                new SwerveModuleIO() {},
                new SwerveModuleIO() {});

        shooter = new Shooter(new ShooterIO() {});
        indexer = new Indexer(new IndexerIO() {});
        // climber = new Climber(new ClimberIO() {});
        hopper = new Hopper(new HopperIO() {});
        intake = new Intake(new IntakeIO() {});
        vision = new Vision(swerve::addVisionMeasurement, new VisionIO() {});
    }

    // Create Operator Interface
    // TODO: Sim operator interface
    operatorInterface = new OperatorInterface(robotState);

    configureAutoModes();
    configureBindings();
  }

  private void configureAutoModes() {}

  private void configureBindings() {
    // Swerve Controls
    swerve.setDefaultCommand(swerve.joystickDrive(operatorInterface.getSwerveControlSignal()));
    operatorInterface.restGyroTrigger().onTrue(Commands.runOnce(() -> swerve.resetGyro()));
    operatorInterface.stopWithXTrigger().whileTrue(swerve.stopWithX());

    // Location Based Commands
    Bounds shotBounds =
        new Bounds(0.0, FieldConstants.LinesVertical.hubCenter, 0.0, FieldConstants.fieldWidth);

    Bounds passLeftBounds =
        new Bounds(
            FieldConstants.LinesVertical.hubCenter,
            FieldConstants.fieldLength,
            FieldConstants.LinesHorizontal.center,
            FieldConstants.fieldWidth);

    Bounds passRightBounds =
        new Bounds(
            FieldConstants.LinesVertical.hubCenter,
            FieldConstants.fieldLength,
            0.0,
            FieldConstants.LinesHorizontal.center);

    Trigger shootHubZoneTrigger =
        new Trigger(
            () -> {
              Bounds allianceBounds = AllianceFlipUtil.apply(shotBounds);
              return allianceBounds.contains(robotState.getRobotPose().getTranslation())
                  && robotState.isEnabled();
            });

    Trigger passLeftZoneTrigger =
        new Trigger(
            () -> {
              Bounds allianceBounds = AllianceFlipUtil.apply(passLeftBounds);
              return allianceBounds.contains(robotState.getRobotPose().getTranslation())
                  && robotState.isEnabled();
            });

    Trigger passRightZoneTrigger =
        new Trigger(
            () -> {
              Bounds allianceBounds = AllianceFlipUtil.apply(passRightBounds);
              return allianceBounds.contains(robotState.getRobotPose().getTranslation())
                  && robotState.isEnabled();
            });

    operatorInterface
        .forceHub()
        .or(shootHubZoneTrigger)
        .and(operatorInterface.trenchMode().negate())
        .whileTrue(shooter.aimAtTarget(TargetLocation.Hub));

    operatorInterface
        .forceFeedLeft()
        .or(passLeftZoneTrigger)
        .and(operatorInterface.trenchMode().negate())
        .whileTrue(shooter.aimAtTarget(TargetLocation.LeftPass));

    operatorInterface
        .forceFeedRight()
        .or(passRightZoneTrigger)
        .and(operatorInterface.trenchMode().negate())
        .whileTrue(shooter.aimAtTarget(TargetLocation.RightPass));

    // Feed Controls
    operatorInterface
        .feedStop()
        .toggleOnTrue(hopper.runTestVoltage().alongWith(indexer.runTestVoltage()));

    // Intake Controls
    operatorInterface.intakeButton().whileTrue(intake.runRollerIntake());
    operatorInterface.extendButton().onTrue(intake.setExtensionDistance(Inches.of(10)));
    operatorInterface.retractButton().onTrue(intake.setExtensionDistance(Inches.of(0)));

    // operatorInterface
    //     .trenchMode()
    //     .whileTrue(shooter.setPose(Degrees.of(0), Degrees.of(0), RotationsPerSecond.of(32)));

    Trigger enableTrigger = new Trigger(() -> robotState.isEnabled());

    enableTrigger.onTrue(
        Commands.runOnce(
            () -> {
              HubShiftUtil.initialize();
              System.out.println("Enabled");
            }));
  }

  public Command getAutonomousCommand() {
    return operatorInterface.getAutoCommand();
  }
}

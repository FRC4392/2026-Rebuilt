// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.DeceiverRobotState.FeederStatus;
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
import frc.robot.subsystems.leds.Leds;
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
import java.util.function.BooleanSupplier;

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
  public final Leds leds;

  // public final AutoFactory autoFactory;

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
    RobotController.setBrownoutVoltage(6.5);

    leds = new Leds();

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

        indexer = new Indexer(new IndexerIOReal());
        // climber = new Climber(new ClimberIOReal());
        hopper = new Hopper(new HopperIOReal());
        intake = new Intake(new IntakeIOReal());
        shooter = new Shooter(new ShooterIOReal(), () -> hopper.getTurretAbsoluteEncoder());
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

        indexer = new Indexer(new IndexerIOSim());
        // climber = new Climber(new ClimberIOSim());
        hopper = new Hopper(new HopperIOSim());
        intake = new Intake(new IntakeIOSim());
        shooter = new Shooter(new ShooterIOSim(), () -> hopper.getTurretAbsoluteEncoder());
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

        indexer = new Indexer(new IndexerIO() {});
        // climber = new Climber(new ClimberIO() {});
        hopper = new Hopper(new HopperIO() {});
        intake = new Intake(new IntakeIO() {});
        vision = new Vision(swerve::addVisionMeasurement, new VisionIO() {});
        shooter = new Shooter(new ShooterIO() {}, () -> hopper.getTurretAbsoluteEncoder());
    }

    // Create Operator Interface
    // TODO: Sim operator interface

    operatorInterface = new OperatorInterface(robotState);

    configureAutoModes();

    configureBindings();

    // Auto Factory
    // autoFactory =
    //     new AutoFactory(swerve::getPose, swerve::setPose, swerve::followTrajectory, true,
    // swerve);
  }

  private void configureAutoModes() {
    new EventTrigger("Intake").whileTrue(intake.runIntakeAuto());
    new EventTrigger("Trench Mode")
        .whileTrue(shooter.setPose(Degrees.of(0), Degrees.of(0), RotationsPerSecond.of(30)));
    new EventTrigger("AimAtHub")
        .whileTrue(
            Commands.deadline(
                shooter.aimAtTarget(TargetLocation.Hub),
                Commands.waitTime(Milliseconds.of(50)).andThen(hopper.runTestVoltage()),
                Commands.waitTime(Milliseconds.of(50)).andThen(indexer.runTestVoltage()),
                Commands.waitSeconds(3).andThen(intake.feedMode())));
  }

  private void configureBindings() {
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

    BooleanSupplier goSlowSupplier =
        () -> {
          return (robotState.getFeederStatus() == FeederStatus.Feeding)
              & !(passLeftBounds.contains(swerve.getPose().getTranslation())
                  || passRightBounds.contains(swerve.getPose().getTranslation()));
        };
    // Swerve Controls
    swerve.setDefaultCommand(
        swerve.joystickDrive(operatorInterface.getSwerveControlSignal(), goSlowSupplier));

    operatorInterface
        .swerveAltMode()
        .whileTrue(
            swerve.joystickDriveAtAngle(
                operatorInterface.getSwerveAngleControlSignal(), goSlowSupplier));

    operatorInterface.restGyroTrigger().onTrue(Commands.runOnce(() -> swerve.resetGyro()));
    operatorInterface.stopWithXTrigger().whileTrue(swerve.stopWithX());

    // RobotModeTriggers.teleop().onTrue(intake.setExtensionDistance(Inches.of(9.9)));
    // Location Based Commands

    Bounds trenchBoundsLeft =
        new Bounds(
            FieldConstants.LinesVertical.hubCenter - 2,
            FieldConstants.LinesVertical.hubCenter + 2,
            FieldConstants.LinesHorizontal.leftTrenchOpenEnd,
            FieldConstants.LinesHorizontal.leftTrenchOpenStart);

    Bounds trenchBoundsRight =
        new Bounds(
            FieldConstants.LinesVertical.hubCenter - 2,
            FieldConstants.LinesVertical.hubCenter + 2,
            FieldConstants.LinesHorizontal.rightTrenchOpenEnd,
            FieldConstants.LinesHorizontal.rightTrenchOpenStart);

    Bounds fippedTrenchLeft = AllianceFlipUtil.apply(trenchBoundsLeft);
    Bounds fippedTrenchRight = AllianceFlipUtil.apply(trenchBoundsRight);

    Trigger temp =
        new Trigger(
            () -> {
              return trenchBoundsLeft.contains(robotState.getRobotPose().getTranslation())
                  || trenchBoundsRight.contains(robotState.getRobotPose().getTranslation())
                  || fippedTrenchLeft.contains(robotState.getRobotPose().getTranslation())
                  || fippedTrenchRight.contains(robotState.getRobotPose().getTranslation());
            });

    Trigger isInTrench = temp.and(operatorInterface.forceShoot().negate());

    Bounds shotBounds =
        new Bounds(0.0, FieldConstants.LinesVertical.hubCenter, 0.0, FieldConstants.fieldWidth);

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
        .and(RobotModeTriggers.teleop())
        .and(isInTrench.negate())
        .whileTrue(shooter.aimAtTarget(TargetLocation.Hub));

    operatorInterface
        .forceFeedLeft()
        .or(passLeftZoneTrigger)
        .and(operatorInterface.trenchMode().negate())
        .and(RobotModeTriggers.teleop())
        .and(isInTrench.negate())
        .whileTrue(shooter.aimAtTarget(TargetLocation.LeftPass));

    operatorInterface
        .forceFeedRight()
        .or(passRightZoneTrigger)
        .and(operatorInterface.trenchMode().negate())
        .and(RobotModeTriggers.teleop())
        .and(isInTrench.negate())
        .whileTrue(shooter.aimAtTarget(TargetLocation.RightPass));

    isInTrench
        .and(RobotModeTriggers.teleop())
        .whileTrue(shooter.setPose(Degrees.of(0), Degrees.of(0), RotationsPerSecond.of(30)));

    // Feed Controls
    operatorInterface
        .feedStop()
        .toggleOnTrue(hopper.runTestVoltage().alongWith(indexer.runTestVoltage()));

    // Intake Controls
    operatorInterface
        .intakeButton()
        .whileTrue(
            intake
                .runRollerIntake()
                .ignoringDisable(true)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    operatorInterface
        .extendButton()
        .and(operatorInterface.intakeButton().negate())
        .onTrue(intake.setExtensionDistance(Inches.of(9.9)));
    operatorInterface
        .retractButton()
        .and(operatorInterface.intakeButton().negate())
        .onTrue(intake.setExtensionDistance(Inches.of(0)));

    operatorInterface
        .feedMode()
        .and(operatorInterface.intakeButton().negate())
        .whileTrue(intake.feedMode());

    operatorInterface
        .trenchMode()
        .whileTrue(shooter.setPose(Degrees.of(0), Degrees.of(0), RotationsPerSecond.of(-32)));

    operatorInterface.vomit().whileTrue(intake.runRollerOuttake().alongWith(hopper.vomit()));

    // HubShiftUtil.setAllianceWinOverride(
    //     () -> Optional.of(operatorInterface.shiftOverride().getAsBoolean()));

    RobotModeTriggers.autonomous().onTrue(intake.setExtensionDistance(Inches.of(9.9)));

    RobotModeTriggers.teleop()
        .onTrue(
            Commands.runOnce(
                () -> {
                  HubShiftUtil.initialize();
                }));
  }

  public Command getAutonomousCommand() {
    return operatorInterface.getAutoCommand();
  }

  // Autos
  // private Command leftTrenchAutoRoutine() {
  //   AutoRoutine routine = autoFactory.newRoutine("autos");

  //   AutoTrajectory leftTrenchTrajectory = routine.trajectory("Left_Path");

  //   routine
  //       .active()
  //       .onTrue(
  //           Commands.sequence(leftTrenchTrajectory.resetOdometry(), leftTrenchTrajectory.cmd()));

  //   return routine.cmd();
  // }

  // private Command rightTrenchAutoRoutine() {
  //   AutoRoutine routine = autoFactory.newRoutine("autos");

  //   AutoTrajectory rightTrenchTrajectory1 = routine.trajectory("Right_Path", 0);
  //   AutoTrajectory rightTrenchTrajectory2 = routine.trajectory("Right_Path", 1);

  //   routine
  //       .active()
  //       .onTrue(
  //           Commands.sequence(
  //               rightTrenchTrajectory1.resetOdometry(),
  //               rightTrenchTrajectory1.cmd(),
  //               Commands.waitSeconds(5),
  //               rightTrenchTrajectory2.cmd()));

  //   return routine.cmd();
  // }
}

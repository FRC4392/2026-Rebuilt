// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.operatorinterface.OperatorInterface;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.climber.ClimberIOSim;
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
import frc.robot.subsystems.vision.VisionIOLimelight;
import java.util.function.Supplier;

public class RobotContainer {
  private final DeceiverRobotState robotState;

  // Subsystems
  public final Swerve swerve;

  public final Shooter shooter;
  public final Indexer indexer;
  public final Hopper hopper;
  public final Climber climber;
  public final Intake intake;
  public final Vision vision;

  // Operator Interface
  private final OperatorInterface operatorInterface;

  /**
   * Constructor
   *
   * @param state RobotState object to track the state of the robot
   */
  public RobotContainer(DeceiverRobotState state) {
    robotState = state;

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
                new SwerveModuleIODeceivers(3),
                state);

        shooter = new Shooter(new ShooterIOReal());
        indexer = new Indexer(new IndexerIOReal());
        climber = new Climber(new ClimberIOReal());
        hopper = new Hopper(new HopperIOReal());
        intake = new Intake(new IntakeIOReal(), state);
        vision =
            new Vision(
                swerve::addVisionMeasurement,
                new VisionIOLimelight("limelight-two", this.gyroTest()),
                new VisionIOLimelight("limelight-three", this.gyroTest()));
        break;
      case SIM:
        // Simulated robot use simulation hardware interfaces
        swerve =
            new Swerve(
                new GyroIO() {},
                new SwerveModuleIOSim(),
                new SwerveModuleIOSim(),
                new SwerveModuleIOSim(),
                new SwerveModuleIOSim(),
                state);

        shooter = new Shooter(new ShooterIOSim());
        indexer = new Indexer(new IndexerIOSim());
        climber = new Climber(new ClimberIOSim());
        hopper = new Hopper(new HopperIOSim());
        intake = new Intake(new IntakeIOSim(), state);
        vision = new Vision(null, null);
        break;
      default:
        // Replay, don't use hardware
        swerve =
            new Swerve(
                new GyroIO() {},
                new SwerveModuleIO() {},
                new SwerveModuleIO() {},
                new SwerveModuleIO() {},
                new SwerveModuleIO() {},
                state);

        shooter = new Shooter(new ShooterIO() {});
        indexer = new Indexer(new IndexerIO() {});
        climber = new Climber(new ClimberIO() {});
        hopper = new Hopper(new HopperIO() {});
        intake = new Intake(new IntakeIO() {}, state);
        vision = new Vision(null, null);
    }

    // Create Operator Interface
    // TODO: Sim operator interface
    operatorInterface = new OperatorInterface(robotState);

    configureAutoModes();
    configureBindings();
  }

  private void configureAutoModes() {}

  private void configureBindings() {
    // swerve.setDefaultCommand(swerve.joystickDrive(operatorInterface.getSwerveControlSignal()));

    operatorInterface.hopperButton().whileTrue(hopper.runTestVoltage());
    // operatorInterface.climberButton().whileTrue(climber.runTestVoltage());
    operatorInterface.indexerButton().whileTrue(indexer.runTestVoltage());
    // operatorInterface
    //     .shooterButton()
    //     .whileTrue(shooter.run(() -> shooter.setShooter(RotationsPerSecond.of(80))));
    // operatorInterface.intakeButton().whileTrue(intake.runRollerIntake());
    // operatorInterface.outtakeButton().whileTrue(intake.runRollerOuttake());
    // operatorInterface.retractButton().whileTrue(intake.runExtensionInManual());
    // operatorInterface.extendButton().whileTrue(intake.runExtensionOutManual());

    // shooter.setDefaultCommand(shooter.setHood(operatorInterface.turretSpeedSupplier()));

    shooter.setDefaultCommand(
        shooter.setPose(Degrees.of(30), Degrees.of(1), RotationsPerSecond.of(37)));

    operatorInterface.extendButton().onTrue(shooter.setHood(Degrees.of(30)));
    operatorInterface.retractButton().onTrue(shooter.setHood(Degrees.of(0)));
    // shooter.setDefaultCommand(shooter.runTurret(operatorInterface.turretSpeedSupplier()));

    // operatorInterface.testLeftTurret().onTrue(shooter.runTurret(Degrees.of(-90)));
    // operatorInterface.testRightTurret().onTrue(shooter.runTurret(Degrees.of(90)));
    // operatorInterface.testUpTurret().onTrue(shooter.runTurret(Degrees.of(0)));
    // operatorInterface.testDownTurret().onTrue(shooter.runTurret(Degrees.of(180)));
  }

  public Command getAutonomousCommand() {
    return operatorInterface.getAutoCommand();
  }

  public Supplier<Rotation2d> gyroTest() {
    return () -> Rotation2d.kCW_90deg;
  }
}

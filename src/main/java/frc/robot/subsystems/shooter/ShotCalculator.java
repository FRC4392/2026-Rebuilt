package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.DeceiverRobotState;
import org.littletonrobotics.junction.Logger;

public class ShotCalculator {

  private ShotCalculator() {
    robotState = DeceiverRobotState.getInstance();
  }

  private static ShotCalculator instance;

  private final DeceiverRobotState robotState;

  private final LinearFilter hoodAngleFilter = LinearFilter.movingAverage(5);
  private final LinearFilter driveAngleFilter = LinearFilter.movingAverage(5);

  private Angle lastHoodAngle;
  private Rotation2d lastTurretAngle;

  public static ShotCalculator getInstance() {
    if (instance == null) {
      instance = new ShotCalculator();
    }
    return instance;
  }

  public record ShotParameters(
      boolean isValid,
      Rotation2d turretAngle,
      AngularVelocity turretVelocity,
      Angle hoodAngle,
      AngularVelocity hoodVelocity,
      AngularVelocity flywheelSpeed,
      Distance distance,
      Distance distanceNoLookAhead,
      Time timeOfFlight,
      boolean passing) {}

  public enum ShotType {
    Shot,
    Pass
  };

  private ShotParameters latestParameters = null;

  // // Presets

  // public static final LaunchPreset passingPreset;
  // public static final LaunchPreset hubPreset;
  // public static final LaunchPreset towerPreset;
  // public static final LaunchPreset trenchPreset;
  // public static final LaunchPreset outpostPreset;
  // public static final LaunchPreset hoodMinPreset =
  // new LaunchPreset(
  // new LoggedTunableNumber(
  // "LaunchCalculator/Presets/HoodMin/HoodAngle",
  // Units.radiansToDegrees(Hood.minAngle)),
  // new LoggedTunableNumber("LaunchCalculator/Presets/HoodMin/FlywheelSpeed",
  // 50));
  // public static final LaunchPreset hoodMaxPreset =
  // new LaunchPreset(
  // new LoggedTunableNumber(
  // "LaunchCalculator/Presets/HoodMax/HoodAngle",
  // Units.radiansToDegrees(Hood.maxAngle)),
  // new LoggedTunableNumber("LaunchCalculator/Presets/HoodMax/FlywheelSpeed",
  // 50));

  // public static final LoggedTunableNumber passingIdleSpeed =
  // new LoggedTunableNumber("LaunchCalculator/PassingIdleSpeed", 100);

  // public static record LaunchPreset(
  // LoggedTunableNumber hoodAngleDeg, LoggedTunableNumber flywheelSpeed) {}

  // // Passing targets
  // private static final double xPassTarget = Units.inchesToMeters(37);
  // private static final double yPassTarget = Units.inchesToMeters(65);

  public double getNaiveTOF(double distance) {
    return timeOfFlightMap.get(distance);
  }

  public void clearLaunchingParameters() {
    latestParameters = null;
  }

  public ShotParameters getParameters(Translation2d target, ShotType shotType) {
    // Calculate estimated pose while accounting for phase delay
    Pose2d estimatedPose = robotState.getRobotPose();
    ChassisSpeeds robotRelativeVelocity = robotState.getRobotSpeeds();
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    Pose2d shooterPostiion = estimatedPose.transformBy(ShooterTransorm);
    double shooterToTargetDistance = target.getDistance(shooterPostiion.getTranslation());

    // Calculate field relative launcher velocity
    var robotVelocity = robotState.getSetpointsSpeeds();

    robotVelocity =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            robotState.getSetpointsSpeeds(), robotState.getRobotPose().getRotation());
    var robotAngle = robotState.getRobotPose().getRotation();

    ChassisSpeeds shooterVelocity =
        transformVelocity(robotVelocity, shooterTranslation, robotAngle);

    // Account for imparted velocity by shooter offset
    double timeOfFlight =
        shotType == ShotType.Pass
            ? passingTimeOfFlightMap.get(shooterToTargetDistance)
            : timeOfFlightMap.get(shooterToTargetDistance);
    Pose2d lookaheadPose = shooterPostiion;
    double lookaheadShooterToTargetDistance = shooterToTargetDistance;

    for (int i = 0; i < 20; i++) {
      timeOfFlight =
          shotType == ShotType.Pass
              ? passingTimeOfFlightMap.get(lookaheadShooterToTargetDistance)
              : timeOfFlightMap.get(lookaheadShooterToTargetDistance);
      double offsetX = shooterVelocity.vxMetersPerSecond * timeOfFlight;
      double offsetY = shooterVelocity.vyMetersPerSecond * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              shooterPostiion.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              shooterPostiion.getRotation());
      lookaheadShooterToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // get turret parameters
    Rotation2d turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
    if (lastTurretAngle == null) {
      lastTurretAngle = turretAngle;
    }
    AngularVelocity turretVelocity =
        RadiansPerSecond.of(
            driveAngleFilter.calculate(turretAngle.minus(lastTurretAngle).getRadians() / 0.02));
    lastTurretAngle = turretAngle;

    // Get hood parameters
    Angle hoodAngle =
        shotType == ShotType.Pass
            ? passingHoodAngleMap.get(lookaheadShooterToTargetDistance).getMeasure()
            : hoodAngleMap.get(shooterToTargetDistance).getMeasure();
    if (lastHoodAngle == null) {
      lastHoodAngle = hoodAngle;
    }
    AngularVelocity hoodVelocity =
        RadiansPerSecond.of(
            hoodAngleFilter.calculate((hoodAngle.minus(lastHoodAngle).in(Radians) / 0.02)));
    lastHoodAngle = hoodAngle;

    // Get flywheel parameters
    AngularVelocity flywheelVelocity =
        shotType == ShotType.Pass
            ? RotationsPerSecond.of(passingFlywheelSpeedMap.get(lookaheadShooterToTargetDistance))
            : RotationsPerSecond.of(flywheelSpeedMap.get(shooterToTargetDistance));

    Logger.recordOutput("Shooter/Shot Calculator/Lookahead Pose", lookaheadPose);

    latestParameters =
        new ShotParameters(
            true,
            turretAngle,
            turretVelocity,
            hoodAngle,
            hoodVelocity,
            flywheelVelocity,
            Meters.of(lookaheadShooterToTargetDistance),
            Meters.of(shooterToTargetDistance),
            Seconds.of(timeOfFlight),
            shotType == ShotType.Pass);

    Logger.recordOutput("Shooter/Shot Calculator/Shot Parameters", shooterToTargetDistance);

    return latestParameters;
  }

  /**
   * Transforms a velocity along a translation.
   *
   * @param velocity The original velocity
   * @param transform The transform to the new position
   * @param currentRotation The current rotation of the robot
   * @return The new velocity
   */
  public static ChassisSpeeds transformVelocity(
      ChassisSpeeds velocity, Translation2d transform, Rotation2d currentRotation) {
    return new ChassisSpeeds(
        velocity.vxMetersPerSecond
            + velocity.omegaRadiansPerSecond
                * (transform.getY() * currentRotation.getCos()
                    - transform.getX() * currentRotation.getSin()),
        velocity.vyMetersPerSecond
            + velocity.omegaRadiansPerSecond
                * (transform.getX() * currentRotation.getCos()
                    - transform.getY() * currentRotation.getSin()),
        velocity.omegaRadiansPerSecond);
  }
}

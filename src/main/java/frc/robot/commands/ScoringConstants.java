package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.vision.VisionConstants;

/** Initial commissioning values; distances refer to robot center and hub center. */
public final class ScoringConstants {
  // User-calibrated radius: eight feet, robot center to hub center.
  public static final double shootingDistanceMeters = 5.0 * 0.3048;
  public static final double shootingDistanceStepMeters = 2.5 * 0.0254;
  // Provisional adjustment limits around the default; validate on the robot during calibration.
  public static final double minimumShootingDistanceMeters = 2.0 * 0.3048;
  public static final double maximumShootingDistanceMeters = 10.0 * 0.3048;
  public static final boolean shooterFacesRear = true;
  // Initial threshold only: measure both roller speeds during successful stationary shots.
  public static final double minimumLauncherRadPerSec = 510.0;
  public static final double distanceToleranceMeters = 0.10;
  public static final double headingToleranceRad = Math.toRadians(2.0);
  public static final double approachHeadingRad = Math.toRadians(15.0);
  public static final double realignHeadingRad = Math.toRadians(25.0);
  public static final double stoppedWheelSpeed = 0.05;
  public static final double settleSeconds = 0.25;
  public static final double maxVisionAgeSeconds = 0.35;
  public static final double acquireTimeoutSeconds = 2.0;
  public static final double maneuverTimeoutSeconds = 8.0;
  public static final double maxLinearSpeed = 1.0;
  public static final double maxLinearAcceleration = 1.0;
  public static final double maxAngularSpeed = 1.5;
  public static final double maxAngularAcceleration = 2.0;
  public static final double distanceKp = 1.5;
  public static final double headingKp = 3.0;

  /** Midpoint of the centered tags on opposite hub faces, in blue-origin coordinates. */
  public static Translation2d hubCenter(Alliance alliance) {
    int first = alliance == Alliance.Blue ? 20 : 4;
    int second = alliance == Alliance.Blue ? 26 : 10;
    var a = VisionConstants.aprilTagLayout.getTagPose(first).orElseThrow().toPose2d();
    var b = VisionConstants.aprilTagLayout.getTagPose(second).orElseThrow().toPose2d();
    return a.getTranslation().plus(b.getTranslation()).div(2.0);
  }

  private ScoringConstants() {}
}

package frc.robot.commands;

import static frc.robot.commands.ScoringConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.subsystems.vision.Vision;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Local scoring assist: driver must provide a clear approach near their own hub. */
public class AlignAndShoot extends Command {
  public enum State {
    ACQUIRE,
    ALIGN,
    APPROACH,
    SETTLE,
    SHOOT,
    ABORT
  }

  private final Drive drive;
  private final Superstructure superstructure;
  private final Vision vision;
  private final Supplier<Optional<DriverStation.Alliance>> alliance;
  private final LoggedNetworkNumber distanceSetting =
      new LoggedNetworkNumber("/SmartDashboard/Scoring/DistanceMeters", shootingDistanceMeters);
  private final double fixedDistance;
  private final boolean rearFacing;
  private Translation2d hub;
  private State state = State.ACQUIRE;
  private String reason = "";
  private double distance;
  private double startedAt;
  private double maneuverStartedAt;
  private double spinStartedAt;
  private double readySince;
  private double lastTime;
  private double commandedV;
  private double commandedOmega;

  public AlignAndShoot(Drive drive, Superstructure superstructure, Vision vision) {
    this(drive, superstructure, vision, DriverStation::getAlliance, Double.NaN, shooterFacesRear);
  }

  /**
   * Explicit calibration/alliance injection for simulation and tests; NaN uses dashboard distance.
   */
  public AlignAndShoot(
      Drive drive,
      Superstructure superstructure,
      Vision vision,
      Supplier<Optional<DriverStation.Alliance>> alliance,
      double distanceMeters,
      boolean rearFacing) {
    this.drive = drive;
    this.superstructure = superstructure;
    this.vision = vision;
    this.alliance = alliance;
    fixedDistance = distanceMeters;
    this.rearFacing = rearFacing;
    addRequirements(drive, superstructure);
  }

  @Override
  public void initialize() {
    state = State.ACQUIRE;
    reason = "Waiting for vision";
    hub = null;
    startedAt = lastTime = maneuverStartedAt = Timer.getFPGATimestamp();
    spinStartedAt = readySince = Double.NaN;
    distance = Double.isNaN(fixedDistance) ? distanceSetting.get() : fixedDistance;
    commandedV = (drive.getLeftVelocityMetersPerSec() + drive.getRightVelocityMetersPerSec()) / 2;
    commandedOmega =
        (drive.getRightVelocityMetersPerSec() - drive.getLeftVelocityMetersPerSec())
            / DriveConstants.trackWidth;
    superstructure.stop();
    if (!Double.isFinite(commandedV) || !Double.isFinite(commandedOmega)) {
      abort("Invalid wheel speed");
    }
    if (!Double.isFinite(distance) || distance <= distanceToleranceMeters) {
      abort("Set calibrated Scoring/DistanceMeters before use");
    }
  }

  @Override
  public void execute() {
    double now = Timer.getFPGATimestamp();
    double dt = MathUtil.clamp(now - lastTime, 0.001, 0.05);
    lastTime = now;
    boolean fresh = vision.hasRecentMeasurement(maxVisionAgeSeconds);
    if (state == State.ACQUIRE) {
      var selectedAlliance = alliance.get();
      if (selectedAlliance.isPresent() && fresh) {
        hub = hubCenter(selectedAlliance.get());
        state = State.ALIGN;
        maneuverStartedAt = spinStartedAt = now;
      } else if (now - startedAt >= acquireTimeoutSeconds) {
        abort(selectedAlliance.isEmpty() ? "Alliance unavailable" : "No recent accepted vision");
      }
    } else if (state != State.ABORT && !fresh) {
      abort("Vision stale or disconnected");
    }
    if (state != State.ACQUIRE
        && state != State.SHOOT
        && state != State.ABORT
        && now - maneuverStartedAt >= maneuverTimeoutSeconds) {
      abort("Alignment timed out");
    }
    if (state == State.ABORT) {
      drive.stop();
      superstructure.stop();
      log(false);
      return;
    }
    if (state == State.ACQUIRE) {
      applyDrive(0, 0, dt);
      superstructure.stop();
      log(false);
      return;
    }

    var pose = drive.getPose();
    var delta = hub.minus(pose.getTranslation());
    double range = delta.getNorm();
    double headingError =
        MathUtil.angleModulus(
            delta.getAngle().getRadians()
                - pose.getRotation().getRadians()
                - (rearFacing ? Math.PI : 0));
    double rangeError = range - distance;
    if (!Double.isFinite(range)
        || range < 0.1
        || !Double.isFinite(headingError)
        || !Double.isFinite(drive.getLeftVelocityMetersPerSec())
        || !Double.isFinite(drive.getRightVelocityMetersPerSec())) {
      abort("Invalid robot pose or wheel speed");
      drive.stop();
      superstructure.stop();
      log(false);
      return;
    }
    boolean positioned =
        Math.abs(rangeError) <= distanceToleranceMeters
            && Math.abs(headingError) <= headingToleranceRad;
    if ((state == State.SHOOT || state == State.SETTLE) && !positioned) {
      if (state == State.SHOOT) maneuverStartedAt = now;
      state = State.ALIGN;
      readySince = Double.NaN;
    }
    if (state == State.ALIGN && Math.abs(headingError) <= approachHeadingRad)
      state = State.APPROACH;
    if (state == State.APPROACH && Math.abs(headingError) > realignHeadingRad) state = State.ALIGN;
    if ((state == State.ALIGN || state == State.APPROACH) && positioned) state = State.SETTLE;

    double v = 0;
    double omega = 0;
    if (state == State.ALIGN || state == State.APPROACH) {
      omega = boundedSpeed(headingError, headingKp, maxAngularSpeed, maxAngularAcceleration);
      if (state == State.APPROACH) {
        double headingScale = MathUtil.clamp(1 - Math.abs(headingError) / realignHeadingRad, 0, 1);
        v =
            boundedSpeed(rangeError, distanceKp, maxLinearSpeed, maxLinearAcceleration)
                * headingScale
                * (rearFacing ? -1 : 1);
      }
    }
    applyDrive(v, omega, dt);
    boolean stopped =
        Math.abs(drive.getLeftVelocityMetersPerSec()) <= stoppedWheelSpeed
            && Math.abs(drive.getRightVelocityMetersPerSec()) <= stoppedWheelSpeed
            && Math.abs(commandedV) < 0.001
            && Math.abs(commandedOmega) < 0.001;
    boolean shooterReady =
        now - spinStartedAt >= SuperstructureConstants.spinUpSeconds
            && superstructure.isLauncherReady(minimumLauncherRadPerSec);
    boolean ready = positioned && stopped && shooterReady;
    if (!ready) {
      readySince = Double.NaN;
      if (state == State.SHOOT) {
        state = State.SETTLE;
        maneuverStartedAt = now;
      }
    } else if (Double.isNaN(readySince)) {
      readySince = now;
    }
    if (state == State.SETTLE && ready && now - readySince >= settleSeconds) state = State.SHOOT;
    boolean feed = state == State.SHOOT && ready;
    superstructure.prepareAndFeed(feed);
    reason =
        feed
            ? "Feeding"
            : !positioned
                ? "Aligning"
                : !stopped ? "Stopping" : !shooterReady ? "Shooter spinning up" : "Settling";
    Logger.recordOutput("Scoring/DistanceErrorMeters", rangeError);
    Logger.recordOutput("Scoring/HeadingErrorRadians", headingError);
    Logger.recordOutput("Scoring/TargetHub", hub);
    log(feed);
  }

  /** Feedback speed capped by the remaining braking distance and configured speed limit. */
  private static double boundedSpeed(
      double error, double kp, double maxSpeed, double acceleration) {
    return Math.copySign(
        Math.min(
            kp * Math.abs(error),
            Math.min(maxSpeed, Math.sqrt(2 * acceleration * Math.abs(error)))),
        error);
  }

  private void applyDrive(double v, double omega, double dt) {
    commandedV +=
        MathUtil.clamp(v - commandedV, -maxLinearAcceleration * dt, maxLinearAcceleration * dt);
    commandedOmega +=
        MathUtil.clamp(
            omega - commandedOmega, -maxAngularAcceleration * dt, maxAngularAcceleration * dt);
    drive.runClosedLoop(new ChassisSpeeds(commandedV, 0, commandedOmega));
  }

  private void abort(String message) {
    state = State.ABORT;
    reason = message;
  }

  private void log(boolean feed) {
    Logger.recordOutput("Scoring/State", state.toString());
    Logger.recordOutput("Scoring/Reason", reason);
    Logger.recordOutput("Scoring/Feeding", feed);
    Logger.recordOutput("Scoring/VisionAgeSeconds", vision.getAcceptedMeasurementAgeSeconds());
    Logger.recordOutput("Scoring/DistanceSetpointMeters", distance);
  }

  public State getState() {
    return state;
  }

  @Override
  public boolean isFinished() {
    return state == State.ABORT;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    superstructure.stop();
    Logger.recordOutput("Scoring/Feeding", false);
    Logger.recordOutput("Scoring/State", state == State.ABORT ? "ABORT" : "IDLE");
  }
}

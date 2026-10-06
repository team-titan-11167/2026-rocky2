package frc.robot.commands;

import static frc.robot.commands.ScoringConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.superstructure.*;
import frc.robot.subsystems.vision.*;
import java.util.Optional;
import org.junit.jupiter.api.*;

class AlignAndShootTest {
  private Drive drive;
  private Vision vision;
  private Superstructure shooter;
  private RecordingDriveIO driveIO;
  private RecordingShooterIO shooterIO;
  private VisionIO.PoseObservation[] observations = new VisionIO.PoseObservation[0];
  private boolean connected = true;
  private AlignAndShoot command;

  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  @BeforeEach
  void setup() {
    SimHooks.pauseTiming();
    driveIO = new RecordingDriveIO();
    drive = new Drive(driveIO, new GyroIO() {});
    shooterIO = new RecordingShooterIO();
    shooter = new Superstructure(shooterIO);
    vision =
        new Vision(
            (pose, timestamp, stdDevs) -> {},
            new VisionIO() {
              @Override
              public void updateInputs(VisionIOInputs inputs) {
                inputs.connected = connected;
                inputs.poseObservations = observations;
              }
            });
    place(Alliance.Blue, shootingDistanceMeters, 0);
    command = create(Alliance.Blue, false);
  }

  @AfterEach
  void cleanup() {
    command.end(true);
    CommandScheduler.getInstance().unregisterSubsystem(drive, shooter, vision);
    SimHooks.resumeTiming();
  }

  private AlignAndShoot create(Alliance alliance, boolean rear) {
    return new AlignAndShoot(
        drive, shooter, vision, () -> Optional.of(alliance), shootingDistanceMeters, rear);
  }

  private void place(Alliance alliance, double range, double heading) {
    var hub = hubCenter(alliance);
    drive.setPose(new Pose2d(hub.getX() - range, hub.getY(), new Rotation2d(heading)));
  }

  private void frame(double timestamp, double ambiguity) {
    observations =
        new VisionIO.PoseObservation[] {
          new VisionIO.PoseObservation(
              timestamp,
              new Pose3d(drive.getPose()),
              ambiguity,
              1,
              2.0,
              VisionIO.PoseObservationType.PHOTONVISION)
        };
    vision.periodic();
  }

  private void tick(boolean fresh) {
    SimHooks.stepTiming(0.02);
    drive.periodic();
    shooter.periodic();
    if (fresh) frame(Timer.getFPGATimestamp(), 0.05);
    else {
      observations = new VisionIO.PoseObservation[0];
      vision.periodic();
    }
    command.execute();
  }

  private void reachShooting() {
    command.initialize();
    for (int i = 0; i < 60; i++) tick(true);
    assertEquals(AlignAndShoot.State.SHOOT, command.getState());
    assertNotEquals(0.0, shooterIO.feeder);
  }

  @Test
  void waitsForSpinupAndDwellAndStopsEverythingOnCancel() {
    command.initialize();
    for (int i = 0; i < 25; i++) {
      tick(true);
      assertEquals(0.0, shooterIO.feeder);
    }
    for (int i = 0; i < 30; i++) tick(true);
    assertEquals(AlignAndShoot.State.SHOOT, command.getState());
    assertTrue(command.getRequirements().containsAll(java.util.Set.of(drive, shooter)));
    command.end(true);
    assertEquals(0, shooterIO.feeder);
    assertEquals(0, shooterIO.launcher);
    assertEquals(0, driveIO.left);
    assertEquals(0, driveIO.right);
  }

  @Test
  void rotatingInPlaceImmediatelyStopsFeedingEvenWithZeroAverageSpeed() {
    reachShooting();
    driveIO.measuredLeft = -0.2;
    driveIO.measuredRight = 0.2;
    tick(true);
    assertEquals(0, shooterIO.feeder);
    assertEquals(AlignAndShoot.State.SETTLE, command.getState());
  }

  @Test
  void slowShooterOrDisplacementImmediatelyStopsFeedingAndRequiresNewDwell() {
    reachShooting();
    shooterIO.rightSpeed = 0;
    tick(true);
    assertEquals(0, shooterIO.feeder);
    shooterIO.rightSpeed = 400;
    tick(true);
    assertEquals(0, shooterIO.feeder);
    for (int i = 0; i < 20; i++) tick(true);
    assertNotEquals(0, shooterIO.feeder);
    place(Alliance.Blue, shootingDistanceMeters + 0.5, 0);
    tick(true);
    assertEquals(0, shooterIO.feeder);
    assertEquals(AlignAndShoot.State.APPROACH, command.getState());
  }

  @Test
  void staleOrDisconnectedVisionAbortsAndCutsFeeding() {
    reachShooting();
    for (int i = 0; i < 20; i++) tick(false);
    assertTrue(command.isFinished());
    assertEquals(0, shooterIO.feeder);
    assertEquals(0, shooterIO.launcher);
    command.initialize();
    for (int i = 0; i < 60; i++) tick(true);
    connected = false;
    tick(false);
    assertEquals(AlignAndShoot.State.ABORT, command.getState());
    assertEquals(0, shooterIO.feeder);
  }

  @Test
  void rejectedOldAndFutureFramesDoNotEstablishReadiness() {
    command.initialize();
    frame(Timer.getFPGATimestamp(), 0.9);
    assertFalse(vision.hasRecentMeasurement(maxVisionAgeSeconds));
    frame(Timer.getFPGATimestamp() + 5, 0.05);
    assertFalse(vision.hasRecentMeasurement(maxVisionAgeSeconds));
    frame(Timer.getFPGATimestamp() - 1, 0.05);
    assertFalse(vision.hasRecentMeasurement(maxVisionAgeSeconds));
    for (int i = 0; i < 110; i++) tick(false);
    assertTrue(command.isFinished());
    assertEquals(0, shooterIO.feeder);
  }

  @Test
  void turnsBeforeTranslatingAndTimesOutWithoutFeeding() {
    place(Alliance.Blue, shootingDistanceMeters + 1, Math.PI / 2);
    command.initialize();
    tick(true);
    assertEquals(AlignAndShoot.State.ALIGN, command.getState());
    assertTrue(driveIO.left > 0);
    assertEquals(-driveIO.left, driveIO.right, 1e-9);
    for (int i = 0; i < 410; i++) tick(true);
    assertTrue(command.isFinished());
    assertEquals(0, shooterIO.feeder);
  }

  @Test
  void rangeControlSupportsBackingUpRearShooterAndRedAlliance() {
    place(Alliance.Blue, shootingDistanceMeters - 0.5, 0);
    command.initialize();
    tick(true);
    assertTrue(driveIO.left < 0 && driveIO.right < 0);
    command.end(true);
    command = create(Alliance.Red, true);
    place(Alliance.Red, shootingDistanceMeters + 0.5, Math.PI);
    command.initialize();
    tick(true);
    assertEquals(AlignAndShoot.State.APPROACH, command.getState());
    assertTrue(driveIO.left < 0 && driveIO.right < 0);
  }

  @Test
  void movingHandoffPreservesVelocityAndLimitsDeceleration() {
    driveIO.measuredLeft = driveIO.measuredRight = 0.8;
    drive.periodic();
    place(Alliance.Blue, shootingDistanceMeters + 1, 0);
    command.initialize();
    tick(true);
    assertEquals(0.82, driveIO.left, 1e-6);
    assertEquals(driveIO.left, driveIO.right, 1e-9);
  }

  @Test
  void missingAllianceAndInvalidDistanceNeverDriveOrFeed() {
    command =
        new AlignAndShoot(drive, shooter, vision, Optional::empty, shootingDistanceMeters, false);
    command.initialize();
    for (int i = 0; i < 110; i++) tick(true);
    assertTrue(command.isFinished());
    assertEquals(0, shooterIO.feeder);
    assertEquals(0, driveIO.left);
    command = new AlignAndShoot(drive, shooter, vision, () -> Optional.of(Alliance.Blue), 0, false);
    command.initialize();
    tick(true);
    assertTrue(command.isFinished());
    assertEquals(0, shooterIO.launcher);
  }

  @Test
  void physicsSimulationConvergesFromAngledApproachAndFiresAtRest() {
    CommandScheduler.getInstance().unregisterSubsystem(drive);
    drive = new Drive(new DriveIOSim(), new GyroIO() {});
    place(Alliance.Blue, shootingDistanceMeters + 1, Math.PI + 0.5);
    command = create(Alliance.Blue, true);
    command.initialize();
    for (int i = 0; i < 400 && command.getState() != AlignAndShoot.State.SHOOT; i++) tick(true);
    assertEquals(AlignAndShoot.State.SHOOT, command.getState());
    assertEquals(
        shootingDistanceMeters,
        drive.getPose().getTranslation().getDistance(hubCenter(Alliance.Blue)),
        distanceToleranceMeters);
    assertTrue(Math.abs(drive.getLeftVelocityMetersPerSec()) <= stoppedWheelSpeed);
    assertTrue(Math.abs(drive.getRightVelocityMetersPerSec()) <= stoppedWheelSpeed);
  }

  private static class RecordingDriveIO implements DriveIO {
    double left, right, measuredLeft, measuredRight;

    @Override
    public void updateInputs(DriveIOInputs inputs) {
      inputs.leftVelocityRadPerSec = measuredLeft / DriveConstants.wheelRadiusMeters;
      inputs.rightVelocityRadPerSec = measuredRight / DriveConstants.wheelRadiusMeters;
    }

    @Override
    public void setVelocity(double leftRad, double rightRad, double leftFF, double rightFF) {
      left = leftRad * DriveConstants.wheelRadiusMeters;
      right = rightRad * DriveConstants.wheelRadiusMeters;
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
      left = leftVolts;
      right = rightVolts;
    }
  }

  private static class RecordingShooterIO implements SuperstructureIO {
    double feeder, launcher, rightSpeed = 400;

    @Override
    public void updateInputs(SuperstructureIOInputs inputs) {
      inputs.leftIntakeLauncherVelocityRadPerSec = -400;
      inputs.rightIntakeLauncherVelocityRadPerSec = rightSpeed;
    }

    @Override
    public void setFeederVoltage(double volts) {
      feeder = volts;
    }

    @Override
    public void setIntakeLauncherVoltage(double volts) {
      launcher = volts;
    }
  }
}

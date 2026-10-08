package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

class VisionTest {
  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  @Test
  void logsAllDetectionsAndRoundTripsInputsEvenWithoutUsablePose() {
    var io = new VisionIOPhotonVision("DetectionTest", VisionConstants.robotToCamera0);
    try {
      var inputs = new VisionIOInputsAutoLogged();
      var unknown = target(999, 0.1);
      var known = target(1, 0.9);
      io.processResults(
          inputs,
          List.of(new PhotonPipelineResult(1, 1000000, 1020000, 0, List.of(unknown, known))));
      assertArrayEquals(new int[] {1, 999}, inputs.tagIds);
      assertEquals(0, inputs.poseObservations.length);

      // A rejected single-tag pose still retains its ID and raw pose in replay inputs.
      io.processResults(
          inputs, List.of(new PhotonPipelineResult(2, 2000000, 2020000, 0, List.of(known))));
      assertEquals(1, inputs.poseObservations.length);
      var table = new LogTable(0);
      inputs.toLog(table);
      var replay = new VisionIOInputsAutoLogged();
      replay.fromLog(table);
      assertArrayEquals(new int[] {1}, replay.tagIds);
      assertArrayEquals(inputs.poseObservations, replay.poseObservations);
      assertEquals(2.0, replay.poseObservations[0].timestamp(), 1e-9);
      io.processResults(inputs, List.of());
      assertEquals(0, inputs.tagIds.length);
      assertEquals(0, inputs.poseObservations.length);
    } finally {
      io.camera.close();
    }
  }

  @Test
  void recoversRobotPoseUsingCameraOffsetAndPitch() {
    var io = new VisionIOPhotonVision("TransformTest", VisionConstants.robotToCamera0);
    try {
      var expected = new Pose3d(3.0, 2.0, 0.0, new Rotation3d(0, 0, 0.4));
      var target = target(1, 0.05);
      target.bestCameraToTarget =
          new Transform3d(
              expected.transformBy(VisionConstants.robotToCamera0),
              VisionConstants.aprilTagLayout.getTagPose(1).orElseThrow());
      var inputs = new VisionIOInputsAutoLogged();
      io.processResults(
          inputs, List.of(new PhotonPipelineResult(1, 1000000, 1020000, 0, List.of(target))));
      var actual = inputs.poseObservations[0].pose();
      assertEquals(0.0, expected.getTranslation().getDistance(actual.getTranslation()), 1e-9);
      assertEquals(0.0, expected.getRotation().minus(actual.getRotation()).getAngle(), 1e-9);
    } finally {
      io.camera.close();
    }
  }

  @Test
  void filtersBadPosesAndRetainsFiniteHeadingUncertainty() {
    List<Double> headingStdDevs = new ArrayList<>();
    var vision =
        new Vision(
            (pose, timestamp, stdDevs) -> {
              assertEquals(1.0, timestamp);
              headingStdDevs.add(stdDevs.get(2, 0));
            },
            new VisionIO() {
              @Override
              public void updateInputs(VisionIOInputs inputs) {
                inputs.connected = true;
                inputs.poseObservations =
                    new PoseObservation[] {
                      observation(2, 0, 0.1, 1, 2),
                      observation(2, 0, 0.9, 1, 2),
                      observation(2, 0, -1, 1, 2),
                      observation(-1, 0, 0.1, 2, 2),
                      observation(2, 2, 0.1, 2, 2),
                      observation(Double.NaN, 0, 0.1, 2, 2),
                      observation(2, 0, 0.1, 0, 2),
                      observation(2, 0, 0.1, 2, 0),
                      observation(2, 0, 0.9, 2, 2)
                    };
              }
            });
    try {
      vision.periodic();
      assertEquals(List.of(0.24, 0.12), headingStdDevs);
    } finally {
      CommandScheduler.getInstance().unregisterSubsystem(vision);
    }
  }

  private static PhotonTrackedTarget target(int id, double ambiguity) {
    var target = new PhotonTrackedTarget();
    target.fiducialId = id;
    target.poseAmbiguity = ambiguity;
    target.bestCameraToTarget = new Transform3d(2, 0, 0, new Rotation3d());
    return target;
  }

  private static PoseObservation observation(
      double x, double z, double ambiguity, int count, double distance) {
    return new PoseObservation(
        1,
        new Pose3d(x, 2, z, new Rotation3d()),
        ambiguity,
        count,
        distance,
        PoseObservationType.PHOTONVISION);
  }
}

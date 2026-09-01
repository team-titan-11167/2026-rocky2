package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.simStartingPose;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class DriveSimulationTest {
  private static final double EPSILON = 1e-6;

  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  @Test
  void startsAtConfiguredFieldPose() {
    var io = new DriveIOSim();
    io.updateInputs(new DriveIO.DriveIOInputs());

    assertEquals(simStartingPose, io.getSimulationPose());
  }

  @Test
  void forwardVoltageMovesInPositiveXWithoutTurning() {
    var io = new DriveIOSim();
    var inputs = new DriveIO.DriveIOInputs();
    io.setVoltage(6.0, 6.0);

    for (int i = 0; i < 100; i++) {
      io.updateInputs(inputs);
    }

    Pose2d pose = io.getSimulationPose();
    assertTrue(pose.getX() > simStartingPose.getX() + 0.5);
    assertEquals(simStartingPose.getY(), pose.getY(), EPSILON);
    assertEquals(0.0, pose.getRotation().getRadians(), EPSILON);
    assertTrue(inputs.leftVelocityRadPerSec > 0.0);
    assertTrue(inputs.rightVelocityRadPerSec > 0.0);
  }

  @Test
  void oppositeVoltagesTurnTheRobot() {
    var io = new DriveIOSim();
    var inputs = new DriveIO.DriveIOInputs();
    io.setVoltage(-6.0, 6.0);

    for (int i = 0; i < 50; i++) {
      io.updateInputs(inputs);
    }

    assertTrue(Math.abs(io.getSimulationPose().getRotation().getRadians()) > 0.25);
  }
}

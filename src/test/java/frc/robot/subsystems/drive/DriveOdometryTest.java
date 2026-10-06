package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.Test;

class DriveOdometryTest {
  @Test
  void wheelHeadingAndVisionCorrectionWorkWithoutGyro() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    var io = new WheelIO();
    var drive = new Drive(io, new GyroIO() {});
    try {
      drive.setPose(Pose2d.kZero);
      drive.periodic();
      io.left = 1.0;
      io.right = 1.0;
      SimHooks.stepTiming(0.02);
      drive.periodic();
      assertEquals(1.0, drive.getPose().getX(), 1e-6);
      io.right += DriveConstants.trackWidth * 0.5;
      SimHooks.stepTiming(0.02);
      drive.periodic();
      assertEquals(0.5, drive.getRotation().getRadians(), 1e-6);
      drive.addVisionMeasurement(
          new Pose2d(1.2, 0.1, Rotation2d.fromRadians(0.8)),
          Timer.getFPGATimestamp(),
          VecBuilder.fill(0.01, 0.01, 0.01));
      double corrected = drive.getRotation().getRadians();
      assertTrue(corrected > 0.5 && corrected < 0.8);
      SimHooks.stepTiming(0.02);
      drive.periodic();
      assertEquals(corrected, drive.getRotation().getRadians(), 1e-6);
      drive.setPose(new Pose2d(3, 4, Rotation2d.fromRadians(1.0)));
      SimHooks.stepTiming(0.02);
      drive.periodic();
      assertEquals(1.0, drive.getRotation().getRadians(), 1e-6);
      assertEquals(3.0, drive.getPose().getX(), 1e-6);
    } finally {
      CommandScheduler.getInstance().unregisterSubsystem(drive);
      SimHooks.resumeTiming();
    }
  }

  private static class WheelIO implements DriveIO {
    double left;
    double right;

    @Override
    public void updateInputs(DriveIOInputs inputs) {
      inputs.leftPositionRad = left / DriveConstants.wheelRadiusMeters;
      inputs.rightPositionRad = right / DriveConstants.wheelRadiusMeters;
    }
  }
}

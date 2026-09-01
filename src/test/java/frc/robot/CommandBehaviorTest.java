package frc.robot;

import static frc.robot.subsystems.drive.DriveConstants.wheelRadiusMeters;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureIO;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class CommandBehaviorTest {
  private static final double EPSILON = 1e-9;

  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
  }

  @AfterAll
  static void resumeTiming() {
    SimHooks.resumeTiming();
  }

  @Test
  void mechanismCommandsUseExplicitVoltagesAndStopOnEnd() {
    var io = new RecordingSuperstructureIO();
    var superstructure = new Superstructure(io);

    runOnce(superstructure.intake());
    assertEquals(intakingFeederVoltage, io.feederVolts, EPSILON);
    assertEquals(intakingIntakeVoltage, io.launcherVolts, EPSILON);

    Command eject = superstructure.eject();
    runOnce(eject);
    assertEquals(ejectingFeederVoltage, io.feederVolts, EPSILON);
    assertEquals(ejectingIntakeVoltage, io.launcherVolts, EPSILON);
    eject.end(false);
    assertEquals(0.0, io.feederVolts, EPSILON);
    assertEquals(0.0, io.launcherVolts, EPSILON);
  }

  @Test
  void launchSpinsLauncherBeforeStartingFeeder() {
    var io = new RecordingSuperstructureIO();
    var superstructure = new Superstructure(io);
    Command launch = superstructure.launch();

    launch.initialize();
    launch.execute();
    assertEquals(spinUpFeederVoltage, io.feederVolts, EPSILON);
    assertEquals(launchingLauncherVoltage, io.launcherVolts, EPSILON);

    SimHooks.stepTiming(spinUpSeconds + 0.02);
    launch.execute();
    launch.execute();
    assertEquals(launchingFeederVoltage, io.feederVolts, EPSILON);
    assertEquals(launchingLauncherVoltage, io.launcherVolts, EPSILON);

    launch.end(true);
    assertEquals(0.0, io.feederVolts, EPSILON);
    assertEquals(0.0, io.launcherVolts, EPSILON);
  }

  @Test
  void normalizedDriveUsesClosedLoopWheelSpeedSetpoints() {
    var io = new RecordingDriveIO();
    var drive = new Drive(io, new GyroIO() {});

    DriveCommands.runArcadeNormalized(drive, 0.5, 0.0);

    // WPILib squared arcade input: 0.5 becomes 0.25, then 0.25 * 4 m/s / wheel radius.
    double expectedRadPerSec = 1.0 / wheelRadiusMeters;
    assertEquals(expectedRadPerSec, io.leftRadPerSec, EPSILON);
    assertEquals(expectedRadPerSec, io.rightRadPerSec, EPSILON);
  }

  private static void runOnce(Command command) {
    command.initialize();
    command.execute();
  }

  private static class RecordingSuperstructureIO implements SuperstructureIO {
    double feederVolts;
    double launcherVolts;

    @Override
    public void setFeederVoltage(double volts) {
      feederVolts = volts;
    }

    @Override
    public void setIntakeLauncherVoltage(double volts) {
      launcherVolts = volts;
    }
  }

  private static class RecordingDriveIO implements DriveIO {
    double leftRadPerSec;
    double rightRadPerSec;

    @Override
    public void setVelocity(
        double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
      this.leftRadPerSec = leftRadPerSec;
      this.rightRadPerSec = rightRadPerSec;
    }
  }
}

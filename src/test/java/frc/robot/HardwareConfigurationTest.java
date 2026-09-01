package frc.robot;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import java.util.Set;
import org.junit.jupiter.api.Test;

class HardwareConfigurationTest {
  @Test
  void canIdsMatchRobotAndAreUnique() {
    Set<Integer> ids =
        Set.of(
            leftLeaderCanId,
            leftFollowerCanId,
            rightLeaderCanId,
            rightFollowerCanId,
            feederCanId,
            leftIntakeLauncherCanId,
            rightIntakeLauncherCanId);

    assertEquals(7, ids.size());
    assertEquals(Set.of(2, 3, 4, 5, 6, 7, 8), ids);
  }

  @Test
  void driveGeometryAndLimitsMatchApprovedConfiguration() {
    assertEquals(55, currentLimit);
    assertEquals(10.71, motorReduction);
    assertTrue(leftInverted);
    assertFalse(rightInverted);
    assertEquals(40, feederCurrentLimit);
    assertEquals(60, intakeLauncherCurrentLimit);
  }
}

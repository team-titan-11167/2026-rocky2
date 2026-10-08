package frc.robot.commands;

import static frc.robot.commands.ScoringConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.junit.jupiter.api.*;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

class ShootingDistanceTest {
  private LoggedNetworkNumber input;
  private ShootingDistance distance;
  private XboxControllerSim controller;

  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  @BeforeEach
  void setup() {
    DriverStationSim.resetData();
    var xbox = new CommandXboxController(1);
    controller = new XboxControllerSim(xbox.getHID());
    controller.setPOV(-1);
    input = new LoggedNetworkNumber("/Test/Scoring/DistanceMeters", shootingDistanceMeters);
    input.set(shootingDistanceMeters);
    distance = new ShootingDistance(input);
    distance.bindControls(xbox);
    tick();
  }

  @AfterEach
  void cleanup() {
    var scheduler = CommandScheduler.getInstance();
    scheduler.cancelAll();
    scheduler.getDefaultButtonLoop().clear();
    scheduler.unregisterSubsystem(distance);
    DriverStationSim.resetData();
  }

  private void tick() {
    // Emulate the LoggedRobot input refresh before the scheduler in this unit test.
    input.periodic();
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
    CommandScheduler.getInstance().run();
  }

  private void press(int angle) {
    controller.setPOV(angle);
    tick();
    controller.setPOV(-1);
    tick();
  }

  @Test
  void dpadStepsOncePerPressAndResetsWhileDisabled() {
    controller.setPOV(0);
    for (int i = 0; i < 10; i++) tick();
    assertEquals(shootingDistanceMeters + shootingDistanceStepMeters, distance.getMeters(), 1e-9);
    assertEquals("8 ft 3 in", SmartDashboard.getString("Scoring/Selected shooting distance", ""));
    controller.setPOV(-1);
    tick();
    press(180);
    assertEquals(shootingDistanceMeters, distance.getMeters(), 1e-9);
    press(180);
    assertEquals("7 ft 9 in", SmartDashboard.getString("Scoring/Selected shooting distance", ""));
    press(270);
    assertEquals(shootingDistanceMeters, distance.getMeters(), 1e-9);
    assertTrue(distance.adjustCommand(shootingDistanceStepMeters).getRequirements().isEmpty());
  }

  @Test
  void clampsControllerAndDashboardAndRecoversFromNonfiniteValues() {
    for (int i = 0; i < 20; i++) press(0);
    assertEquals(maximumShootingDistanceMeters, distance.getMeters(), 1e-9);
    for (int i = 0; i < 30; i++) press(180);
    assertEquals(minimumShootingDistanceMeters, distance.getMeters(), 1e-9);
    input.set(100);
    tick();
    assertEquals(maximumShootingDistanceMeters, distance.getMeters(), 1e-9);
    input.set(-1);
    tick();
    assertEquals(minimumShootingDistanceMeters, distance.getMeters(), 1e-9);
    for (double invalid :
        new double[] {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
      input.set(invalid);
      tick();
      assertEquals(shootingDistanceMeters, distance.getMeters(), 1e-9);
      tick();
      assertEquals(shootingDistanceMeters, input.get(), 1e-9);
    }
  }
}

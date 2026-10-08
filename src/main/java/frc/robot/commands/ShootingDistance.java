package frc.robot.commands;

import static frc.robot.commands.ScoringConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Shared, replayable distance selection; an active shot captures its own copy. */
public class ShootingDistance extends SubsystemBase {
  private final LoggedNetworkNumber setting;

  public ShootingDistance() {
    this(new LoggedNetworkNumber("/SmartDashboard/Scoring/DistanceMeters", shootingDistanceMeters));
  }

  ShootingDistance(LoggedNetworkNumber setting) {
    this.setting = setting;
  }

  public double getMeters() {
    double value = setting.get();
    return Double.isFinite(value)
        ? MathUtil.clamp(value, minimumShootingDistanceMeters, maximumShootingDistanceMeters)
        : shootingDistanceMeters;
  }

  public void bindControls(CommandXboxController controller) {
    controller.povUp().onTrue(adjustCommand(shootingDistanceStepMeters));
    controller.povDown().onTrue(adjustCommand(-shootingDistanceStepMeters));
    controller.povLeft().onTrue(resetCommand());
  }

  public Command adjustCommand(double deltaMeters) {
    // No mechanism requirements: adjusting the next shot must not cancel the active shot.
    return Commands.runOnce(
            () ->
                setting.set(
                    MathUtil.clamp(
                        getMeters() + deltaMeters,
                        minimumShootingDistanceMeters,
                        maximumShootingDistanceMeters)))
        .ignoringDisable(true);
  }

  public Command resetCommand() {
    return Commands.runOnce(() -> setting.set(shootingDistanceMeters)).ignoringDisable(true);
  }

  @Override
  public void periodic() {
    double meters = getMeters();
    // Repair invalid dashboard edits as well as bounding controller adjustments.
    if (Double.compare(setting.get(), meters) != 0) setting.set(meters);
    Logger.recordOutput("Scoring/SelectedDistanceMeters", meters);
    Logger.recordOutput("Scoring/SelectedDistanceFeet", Units.metersToFeet(meters));
    SmartDashboard.putString("Scoring/Selected shooting distance", formatDistance(meters));
  }

  public static String formatDistance(double meters) {
    long inches = Math.round(Units.metersToInches(meters));
    return inches / 12 + " ft " + inches % 12 + " in";
  }
}

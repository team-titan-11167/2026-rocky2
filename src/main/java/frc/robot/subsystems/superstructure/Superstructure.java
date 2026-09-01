// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD license.

package frc.robot.subsystems.superstructure;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Superstructure extends SubsystemBase {
  private final SuperstructureIO io;
  private final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged();

  private final LoggedNetworkNumber intakeFeederVolts =
      new LoggedNetworkNumber(
          "/SmartDashboard/Intaking feeder roller value", intakingFeederVoltage);
  private final LoggedNetworkNumber intakeLauncherVolts =
      new LoggedNetworkNumber(
          "/SmartDashboard/Intaking intake roller value", intakingIntakeVoltage);
  private final LoggedNetworkNumber launchFeederVolts =
      new LoggedNetworkNumber(
          "/SmartDashboard/Launching feeder roller value", launchingFeederVoltage);
  private final LoggedNetworkNumber launchLauncherVolts =
      new LoggedNetworkNumber(
          "/SmartDashboard/Launching launcher roller value", launchingLauncherVoltage);
  private final LoggedNetworkNumber spinUpFeederVolts =
      new LoggedNetworkNumber("/SmartDashboard/Spin-up feeder roller value", spinUpFeederVoltage);

  public Superstructure(SuperstructureIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Superstructure", inputs);
  }

  /** Runs both mechanisms at explicit voltages until the command ends. */
  public Command runRollers(double feederVolts, double launcherVolts) {
    return runRollers(() -> feederVolts, () -> launcherVolts);
  }

  private Command runRollers(DoubleSupplier feederVolts, DoubleSupplier launcherVolts) {
    return runEnd(
        () -> {
          io.setFeederVoltage(feederVolts.getAsDouble());
          io.setIntakeLauncherVoltage(launcherVolts.getAsDouble());
        },
        this::stop);
  }

  /** Intake fuel using replay-safe dashboard tuning values. */
  public Command intake() {
    return runEnd(
        () -> {
          io.setFeederVoltage(intakeFeederVolts.get());
          io.setIntakeLauncherVoltage(intakeLauncherVolts.get());
        },
        this::stop);
  }

  /** Eject fuel out of the intake. */
  public Command eject() {
    return runRollers(ejectingFeederVoltage, ejectingIntakeVoltage);
  }

  /** Spin up for one second and then launch until interrupted. */
  public Command launch() {
    return launchSequence(launchLauncherVolts);
  }

  /** The old far-launch setting is currently identical to normal launch. */
  public Command launchFar() {
    return launchSequence(() -> launchingLauncherFarVoltage);
  }

  /** Launch immediately without the spin-up delay, as used by lawn-mower mode. */
  public Command launchImmediately(boolean far) {
    DoubleSupplier launcherVolts = far ? () -> launchingLauncherFarVoltage : launchLauncherVolts;
    return runRollers(launchFeederVolts, launcherVolts);
  }

  private Command launchSequence(DoubleSupplier launcherVolts) {
    return runRollers(spinUpFeederVolts, launcherVolts)
        .withTimeout(spinUpSeconds)
        .andThen(runRollers(launchFeederVolts, launcherVolts));
  }

  public void stop() {
    io.setFeederVoltage(0.0);
    io.setIntakeLauncherVoltage(0.0);
  }
}

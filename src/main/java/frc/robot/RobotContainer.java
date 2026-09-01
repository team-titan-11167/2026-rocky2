// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD license.

package frc.robot;

import static frc.robot.Constants.OperatorConstants.*;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOSpark;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureIO;
import frc.robot.subsystems.superstructure.SuperstructureIOSim;
import frc.robot.subsystems.superstructure.SuperstructureIOSpark;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Declares robot hardware, commands, controls, and autonomous routines. */
public class RobotContainer {
  private final Drive drive;
  private final Superstructure superstructure;

  private final CommandXboxController driverController =
      new CommandXboxController(driverControllerPort);
  private final CommandXboxController operatorController =
      new CommandXboxController(operatorControllerPort);

  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto choices");

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        drive = new Drive(new DriveIOSpark(), new GyroIO() {});
        superstructure = new Superstructure(new SuperstructureIOSpark());
        break;
      case SIM:
        drive = new Drive(new DriveIOSim(), new GyroIO() {});
        superstructure = new Superstructure(new SuperstructureIOSim());
        break;
      default:
        drive = new Drive(new DriveIO() {}, new GyroIO() {});
        superstructure = new Superstructure(new SuperstructureIO() {});
        break;
    }

    NamedCommands.registerCommand("Launch", superstructure.launch().withTimeout(6.0));
    configureAutonomous();
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.arcadeDriveNormalized(
            drive,
            () -> {
              double value = -driverController.getLeftY() * driveScaling;
              return driverController.leftBumper().getAsBoolean()
                  ? value
                  : MathUtil.clamp(value, -0.75, 0.75);
            },
            () -> {
              double value = driverController.getRightX() * rotationScaling;
              return driverController.leftBumper().getAsBoolean()
                  ? value
                  : MathUtil.clamp(value, -0.8, 0.8);
            }));

    operatorController.leftBumper().whileTrue(superstructure.intake());
    operatorController.rightBumper().whileTrue(superstructure.launch());
    operatorController.a().whileTrue(superstructure.eject());
    operatorController.b().whileTrue(superstructure.launchFar());
    operatorController.x().whileTrue(superstructure.launchImmediately(true));
    driverController.rightBumper().whileTrue(wiggler());

    superstructure.setDefaultCommand(superstructure.run(superstructure::stop));
  }

  private void configureAutonomous() {
    autoChooser.addDefaultOption("Autonomous", exampleAuto());
    autoChooser.addOption("Short", shortAuto());
    autoChooser.addOption("YOLO", lawnMowerMode());
    autoChooser.addOption("Right Bespoke", bespokeAuto(false));
    autoChooser.addOption("Left Bespoke", bespokeAuto(true));

    // Keep the AdvantageKit template's characterization tools available for closed-loop tuning.
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  private Command autoDrive(double xSpeed, double zRotation) {
    return DriveCommands.arcadeDriveNormalized(drive, () -> xSpeed, () -> zRotation);
  }

  private Command exampleAuto() {
    return Commands.sequence(
        autoDrive(0.5, 0.0).withTimeout(0.9),
        superstructure.launch().withTimeout(8.0),
        autoDrive(-0.5, 0.0).withTimeout(0.1),
        autoDrive(0.5, 0.0).withTimeout(0.1),
        autoDrive(-0.5, 0.0).withTimeout(0.1),
        superstructure.launchImmediately(false).withTimeout(7.0));
  }

  private Command shortAuto() {
    return Commands.sequence(
        autoDrive(0.5, 0.0).withTimeout(0.8),
        superstructure.launch().withTimeout(8.0),
        autoDrive(0.0, 0.5).withTimeout(0.25),
        autoDrive(0.5, 0.0).withTimeout(3.0));
  }

  private Command bespokeAuto(boolean left) {
    double firstTurn = left ? 0.5 : -0.5;
    return Commands.sequence(
        autoDrive(0.5, 0.0).withTimeout(0.9),
        superstructure.launch().withTimeout(4.0),
        autoDrive(-0.5, 0.0).withTimeout(0.1),
        autoDrive(0.5, 0.0).withTimeout(0.1),
        autoDrive(-0.5, 0.0).withTimeout(0.1),
        superstructure.launchImmediately(false).withTimeout(3.0),
        autoDrive(0.0, firstTurn).withTimeout(0.5),
        Commands.deadline(superstructure.intake().withTimeout(3.0), autoDrive(0.5, 0.0)),
        autoDrive(0.0, -firstTurn).withTimeout(0.9),
        Commands.deadline(superstructure.intake().withTimeout(3.0), autoDrive(0.5, 0.0)));
  }

  private Command lawnMowerMode() {
    return Commands.parallel(superstructure.launchImmediately(true), autoDrive(0.5, 0.0));
  }

  private Command wiggler() {
    return Commands.sequence(
        autoDrive(-0.5, 0.0).withTimeout(0.1),
        autoDrive(0.5, 0.0).withTimeout(0.1),
        autoDrive(-0.5, 0.0).withTimeout(0.1));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}

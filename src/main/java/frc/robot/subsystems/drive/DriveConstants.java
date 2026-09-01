// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = 4.0;
  public static final double trackWidth = Units.inchesToMeters(26.0);
  public static final Pose2d simStartingPose = new Pose2d(2.0, 4.0, Rotation2d.kZero);

  // Device CAN IDs
  public static final int leftLeaderCanId = 3;
  public static final int leftFollowerCanId = 4;
  public static final int rightLeaderCanId = 6;
  public static final int rightFollowerCanId = 5;

  // Motor configuration
  public static final int currentLimit = 55;
  public static final double wheelRadiusMeters = Units.inchesToMeters(3.0);
  public static final double motorReduction = 10.71;
  public static final boolean leftInverted = true;
  public static final boolean rightInverted = false;
  public static final DCMotor gearbox = DCMotor.getNEO(2);

  // Velocity PID configuration
  public static final double realKp = 0.0;
  public static final double realKd = 0.0;
  public static final double realKs = 0.0;
  public static final double realKv = 12.0 / (gearbox.freeSpeedRadPerSec / motorReduction);

  public static final double simKp = 0.05;
  public static final double simKd = 0.0;
  public static final double simKs = 0.0;
  public static final double simKv = 0.227;

  // PathPlanner configuration
  public static final double robotMassKg = 74.088;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              gearbox.withReduction(motorReduction),
              currentLimit,
              2),
          trackWidth);
}

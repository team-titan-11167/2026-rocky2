# AdvantageKit KitBot Migration Notes

This project ports the robot behavior from
[`rockhillfirstrobotics/FRC2026`](https://github.com/rockhillfirstrobotics/FRC2026/)
commit `6af336722e44b9965b01fbf6692b81636add1386` into the AdvantageKit KitBot
architecture. The deploy team is 11167.

## Hardware map

All seven controllers are REV SPARK MAX controllers configured for brushless motors.

| Mechanism | CAN ID | Configuration |
| --- | ---: | --- |
| Left drive leader | 3 | Inverted, brake, 55 A |
| Left drive follower | 4 | Follows CAN 3, brake, 55 A |
| Right drive leader | 6 | Not inverted, brake, 55 A |
| Right drive follower | 5 | Follows CAN 6, brake, 55 A |
| Left intake/launcher | 2 | Inverted, coast, 60 A |
| Feeder | 7 | Not inverted, coast, 40 A |
| Right intake/launcher | 8 | Not inverted, coast, 60 A |

There is no gyro. Real mode supplies an empty `GyroIO`, so AdvantageKit estimates heading from
the drive encoders. The unused Pigeon 2 implementation from the template was removed.

The retained KitBot model uses 6-inch wheels, a 10.71:1 reduction, a 26-inch track width, and two
NEO motors per side. Robot mass and moment of inertia are still the generic template values and
must be measured before precise PathPlanner work.

## Controls and autonomous behavior

- Driver Xbox controller: port 0. Left Y drives, right X turns, with 0.7 translation and 0.8
  rotation scaling. Right bumper runs the three-step wiggler.
- Operator Xbox controller: port 1. Left bumper intakes, right bumper launches, A ejects, B runs
  far launch, and X runs the launcher immediately in lawn-mower mode.
- The autonomous chooser retains Autonomous, Short, YOLO, Right Bespoke, and Left Bespoke with the
  original sequence values and timings. AdvantageKit feedforward and SysId routines are additional
  chooser entries for drivetrain tuning.
- The old SmartDashboard tuning keys and robot-status keys are retained. Tunable mechanism inputs
  use `LoggedNetworkNumber` so their values are replayable.

The old left-bumper drive branch is retained, but it currently makes no numerical difference: the
0.7 translation and 0.8 rotation scales already fit inside the non-bumper clamps of 0.75 and 0.8.

## Intentional behavior differences

- Teleop and autonomous arcade requests now use closed-loop wheel velocity instead of direct duty
  cycle. Inputs retain WPILib's squared arcade shaping. The initial real feedforward is calculated
  from the NEO free speed and gearing; characterize and tune it on the robot before competition.
- Launcher control now uses volts consistently. The old launcher called `SparkMax.set()` with
  values named as volts. Its `0.25` intake value was effectively about 3 V with 12 V compensation,
  while values of 11 were clamped to full output. The migrated defaults therefore use 3 V for
  intake, -3 V for eject, and 12 V for launching.
- Feeder defaults preserve the observed behavior: +10 V intake, -9 V eject/launch, and 0 V during
  launcher spin-up. The old eject command read the launch dashboard value instead of its dedicated
  eject constant; the migrated code makes the effective -9 V behavior explicit.
- Normal and far launch both use 12 V because the old constants were identical.
- Every mechanism command now explicitly stops both outputs when interrupted. The old launch and
  lawn-mower command classes relied on a later default command to stop outputs.
- AdvantageKit IO logging records each launcher motor separately. The template's duplicated
  right-leader current signal and its use of the feeder constant for intake-launcher output were
  corrected.

## Before driving the real robot

1. Put the robot securely on blocks and confirm CAN IDs 2 through 8 appear exactly once.
2. Run each motor at a low dashboard voltage and confirm direction, inversion, and follower motion.
3. Verify that forward commands move both drive sides forward and positive turn commands match the
   old controller direction.
4. Run the feedforward and SysId chooser routines, then replace the initial drivetrain gains with
   measured values.
5. Measure robot mass, moment of inertia, and track width before trusting generated paths.

## Desktop simulation

The drivetrain simulation now models two NEO motors per side and begins at `(2.0 m, 4.0 m, 0°)` on
the blue end of the field. It publishes encoder odometry at `Odometry/Robot` and physics ground
truth at `Simulation/RobotPose`. See [SIMULATION.md](SIMULATION.md) for keyboard, Xbox controller,
Sim GUI, and AdvantageScope setup.

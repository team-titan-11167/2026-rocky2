# AprilTag positioning

The PhotonVision portions of the [AdvantageKit v26.0.2 vision template](https://github.com/Mechanical-Advantage/AdvantageKit/tree/v26.0.2/template_projects/sources/vision/src/main/java/frc/robot/subsystems/vision) are integrated with the existing differential-drive pose estimator. PhotonLib is pinned to the template's v2026.3.2 dependency. Original source headers and the repository's AdvantageKit license are retained.

## Configuration and commissioning

- Configure the PhotonVision camera name as **Rear Camera**, use a compatible PhotonVision v2026.3.2 coprocessor image, and enable an AprilTag pipeline with 3D estimation and MultiTag solving on the coprocessor. No heading input or IMU is required by this solver.
- Calibrate the camera at the actual pipeline resolution. Check exposure and motion blur while driving, not just while stationary.
- `VisionConstants` contains the camera transform: X = -0.3302 m (13 inches rearward), Y = -0.3048 m (right), Z = +0.1524 m (up), pitch = -17.26 degrees, yaw = 180 degrees. The camera faces rearward and tilts **upward**. Measure from the robot's center between the drive sides, at floor height, to the optical center of the camera. Refine translation, pitch, yaw, and roll before trusting precision positioning.
- The selected layout is **2026 REBUILT Welded**, with the standard blue-origin field coordinates on either alliance. Match the layout in PhotonVision to this code and the actual field; use `k2026RebuiltAndymark` in code if appropriate and change the coprocessor layout too. Practice tags must actually be at their layout positions or use a measured custom layout on both systems.
- Check both encoder distances increase during forward travel, measure travel distance to tune wheel radius/reduction, and calibrate effective track width using measured turns on carpet. Existing 26-inch track width, 6-inch wheels, and 10.71 reduction are retained; being a kitbot does not prove these values match the physical robot.
- Start from a known field pose using `Drive.setPose(...)` when implementing a position-based autonomous routine. The current timed autonomous routines are unchanged; vision measurements gradually correct the estimator rather than forcibly resetting it to the first tag.
- Validate stationary poses at several measured positions/headings, then straight drives and turns with tags visible and obscured. Tune the standard deviations in `VisionConstants` from real logs. Template uncertainty baselines are starting values, not accuracy guarantees.

## Logging and replay

`Logger.processInputs("Vision/Camera0", inputs)` records replayable inputs:

- `TagIds`: sorted unique IDs of **all AprilTags detected in frames received during that robot loop**, including unknown IDs and tags not used in a pose solve. Empty means no detections in newly received frames; it does not mean the camera is disconnected.
- `PoseObservations`: capture timestamps, raw robot poses, ambiguity, tag count, average distance, and solver type, before filtering.
- `Connected` and `LatestTargetObservation`: camera connection and best-target angles from the latest received frame (zero when no new result).

`Vision/Camera0/TagPoses` and `Vision/Summary/TagPoses` show known detected tags on an AdvantageScope field. Unknown IDs remain in inputs but cannot have field poses. `RobotPosesAccepted` and `RobotPosesRejected` show the filter decisions. Accepted here means passed the vision filter; the pose estimator can still ignore a measurement outside its history window. `Odometry/Robot` is the fused robot pose, and `Odometry/UsingWheelHeading` indicates the no-gyro fallback.

Single-tag measurements with ambiguity above 0.2 or unavailable ambiguity are rejected, as are invalid numeric values, nonpositive distance/tag count, poses outside the field, and Z errors above 0.75 m. Position and heading uncertainty increase with distance squared and decrease with the number of tags. PhotonVision heading correction remains enabled, which is essential without a gyro.

Simulation uses the drivetrain physics model's ground-truth pose to generate PhotonVision frames, not the fused pose estimate. Camera simulation properties are template defaults and need adjustment to the actual camera to predict visibility accurately. Ideal drivetrain simulation does not reproduce real carpet scrub, collisions, or wheel slip. Replay uses no camera hardware and restores the logged vision inputs.

## Expectations without an IMU

Wheel heading is integrated as `(right distance - left distance) / track width`. This can provide useful field positioning during smooth driving with frequent, good-quality tag observations. It is much less dependable during skid-steer turns, impacts, wheel slip, or long periods without visible tags. A tank drive scrubs its tires to turn, so its effective track width depends on traction and loading. Encoders cannot detect the robot being pushed sideways. A camera can correct these errors only when it sees suitable tags.

A 5-degree heading error produces about 0.44 m of lateral error over a subsequent 5 m straight drive, even with perfect distance measurements. There is no defensible fixed accuracy estimate until the camera is calibrated, the mounting transform is measured, and driving logs are collected. MultiTag observations generally constrain pose better than a distant or nearly head-on single tag, which may have two plausible solutions.

At only six inches above the floor, test for bumper/mechanism obstruction, other robots blocking the view, and whether elevated tags remain in the camera's vertical field of view at close range. The upward tilt trades nearby/high-tag visibility against distant coverage. Fast turns can also blur tags. Adding a yaw gyro is a high-value improvement: it gives independent heading between camera frames and during occlusion, though vision is still needed to correct translational drift. Until then, treat this as useful localization to validate experimentally before relying on tight autonomous paths.

References: [WPILib differential-drive odometry](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/differential-drive-odometry.html), [PhotonVision 3D tracking and ambiguity](https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/3D-tracking.html), and [AdvantageKit vision template](https://docs.advantagekit.org/getting-started/template-projects/vision-template/).

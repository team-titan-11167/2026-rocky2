# Local scoring assist

Hold **driver A** to run `AlignAndShoot`. Start near your own hub with a clear route. The command selects the Driver Station alliance's hub, points the **rear** launcher at its center, adjusts to **8 feet (2.4384 m) center to center**, settles, and feeds. Release A to stop the mechanisms and return to joystick driving. Other commands requiring the drive or superstructure can interrupt the assist. After an abort or interruption, release and press A again to restart.

This is local distance/heading control, without obstacle avoidance or a field-wide route planner. Hub centers use the midpoint of centered opposing hub-face tags (blue 20/26, red 4/10) from the configured REBUILT layout. Coordinates remain in the blue-origin frame for either alliance. A custom practice layout must include correctly positioned hub tags.

## Behavior

- `ACQUIRE`: reduce existing motion toward zero while waiting up to 2 seconds for an alliance and recent accepted vision.
- `ALIGN`: spin up the launcher, turn toward the hub, and decelerate translation. Start from measured wheel velocities so an already-moving robot does not receive an abrupt zero-speed request.
- `APPROACH`: blend range and heading control once within 15 degrees. Reduce translation as heading error increases; return to turn-first alignment above 25 degrees. The rear-facing robot backs toward the hub when too far away and drives forward when too close.
- Speed requests are proportional to error, capped by a braking-distance envelope and speed limits, then acceleration-limited. Initial limits are 1 m/s, 1 m/s², 1.5 rad/s, and 2 rad/s². This is a conservative local controller, not a time-optimal trajectory planner. Physical braking must be validated on carpet.
- `SETTLE`: request zero wheel speeds. Require range within 0.10 m, heading within 2 degrees, **both** measured wheel speeds below 0.05 m/s, effectively zero requested chassis motion, and both launcher rollers above the readiness threshold. These conditions must remain true for 0.25 seconds; spin-up must also have run for at least the existing 0.5 seconds.
- `SHOOT`: feed only while those checks continue to pass. Readiness loss cuts feeding in the same loop. Position drift resumes alignment; wheel movement or launcher slowdown requires settling again.
- Vision older than 0.35 seconds or a disconnected camera aborts an active maneuver and stops outputs. Frame capture timestamps determine freshness; rejected, future-dated, and repeated old frames do not refresh readiness. Alignment times out after 8 seconds without feeding. Recovery from shooting gets a new 8-second budget.

The command requires both subsystems for its full lifetime. It calls direct subsystem output methods, rather than scheduling competing launch commands. The default joystick command therefore cannot resume while shooting. Vision continues running independently.

## Settings and commissioning

`ScoringConstants.java` contains the limits, tolerances, rear-launcher setting, and initial shooter-readiness threshold. `/SmartDashboard/Scoring/DistanceMeters` defaults to 2.4384; its value is captured when A is pressed, so changing it does not move an active target.

1. Configure PhotonVision's camera name exactly as **Rear Camera**. The robot-to-camera transform is X = -13 inches, Y = -12 inches (robot right), Z = +6 inches, yaw = 180 degrees, pitch = -17.26 degrees (upward tilt). Only rearward position and yaw were changed; verify the retained lateral offset, height, and tilt on the robot. See [VISION.md](VISION.md).
2. Verify the fused pose, hub distance, and rear heading against measured positions. Real hardware still uses wheel-derived heading; no gyro hardware was added.
3. Tune drivetrain velocity feedback and braking on carpet. `DriveConstants.realKp` and `realKd` currently remain zero; feedforward alone does not guarantee the requested acceleration or stopping behavior. Do not interpret passing simulation as validation of physical stopping accuracy.
4. Measure both launcher roller speeds during successful shots. The initial minimum is **300 rad/s (about 2865 RPM)** for each roller's speed magnitude. This is an uncalibrated starting threshold, not a known shot-speed target. The launcher retains its existing voltage control; this change adds a measured-speed feed gate, not an RPM control loop.
5. Verify turning, backing toward the hub, driving away when too close, stopping, button release, and tag loss at conservative speeds before loading fuel. Then tune tolerances against actual shot consistency.

AdvantageKit logs `Scoring/State`, `Reason`, `Feeding`, `TargetHub`, `DistanceSetpointMeters`, `DistanceErrorMeters`, `HeadingErrorRadians`, and `VisionAgeSeconds`. Measure button-to-first-accurate-shot time alongside misses and overshoot.

## Automated checks

`AlignAndShootTest` covers spin-up and settling, cancellation, individual wheel motion, launcher slowdown, displacement, stale/disconnected/rejected vision, timeout, missing alliance, invalid distance, forward/rear direction, red alliance selection, moving handoff, and a rear-facing angled approach using drivetrain physics simulation. The simulation supplies ideal accepted localization frames and does not model real camera occlusion or wheel slip.

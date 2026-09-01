# Driving the KitBot in Simulation

The desktop simulation uses the same driver commands as the real robot, a WPILib differential
drivetrain physics model, and AdvantageKit telemetry. WPILib's Sim GUI provides the simulated
Driver Station and joystick input; AdvantageScope provides the live field visualization.

## Start the simulator

The easiest workflow in WPILib VS Code is:

1. Open the command palette and run `WPILib: Simulate Robot Code`.
2. Select **Sim GUI** when the extension picker appears.
3. In the Sim GUI's **Robot State** window, choose **Teleoperated** and click **Enable**.

The equivalent command-line launch is:

```text
./gradlew simulateJava -PsimGui
```

Omit `-PsimGui` for headless simulation or AdvantageKit replay. Do not use VS Code's ordinary Java
run button; it does not prepare WPILib simulation correctly.

## Drive with the keyboard

The committed `simgui-ds.json` assigns Keyboard 0 to driver port 0 and Keyboard 1 to operator port
1. Click the Sim GUI window before using the keys.

| Action | Key |
| --- | --- |
| Drive forward / reverse | W / S |
| Turn left / right | Left Arrow / Right Arrow |
| Driver left bumper | E |
| Driver right bumper (wiggler) | U |
| Operator A (eject) | 1 |
| Operator B (far launch) | 2 |
| Operator X (lawn-mower launcher) | 3 |
| Operator left bumper (intake) | 5 |
| Operator right bumper (launch) | 6 |

W maps to negative Xbox left-Y, matching a real Xbox controller pushed forward. The intake side is
the robot's positive-X/front side and moves forward when W is held.

## Drive with an Xbox controller

1. Connect the controller before enabling simulation.
2. In Sim GUI, drag it from **System Joysticks** onto joystick slot 0, replacing Keyboard 0.
3. Enable **Map gamepad** under **Joysticks** so WPILib applies the normal Xbox axis mapping.
4. Put an optional second controller in slot 1 for the operator controls.

If the controller is connected after the robot is enabled, disable the simulated robot before
assigning it and then enable again.

## View the robot in AdvantageScope

1. Start the robot simulation first.
2. In AdvantageScope, use **File → Connect to Simulator** (`Ctrl+Shift+K`).
3. Add a **3D Field** tab and select the 2026 REBUILT field.
4. Drag `Odometry/Robot` from the sidebar into the tab's **Poses** list.
5. Open that pose's style menu and select the downloaded **2026 KitBot** robot model.
6. Optionally drag `Simulation/RobotPose` into the same tab and render it as a differently colored
   ghost. This compares encoder odometry against the physics model's ground truth.
7. Right-click the field to select **Orbit Robot** or **Driver Station** camera mode.

The 2026 field and KitBot models are downloaded automatically by AdvantageScope. If the model is
missing, open **App → Asset Download Status**, connect to the internet, and allow the detailed 2026
assets to finish downloading. The built-in generic KitBot model is a usable offline fallback.

## What is simulated

- Two NEO motors per drive side, 10.71:1 gearing, six-inch wheels, and differential-drive dynamics.
- Closed-loop wheel velocity, encoder position/velocity, current draw, drivetrain pose, and all
  normal AdvantageKit logs.
- Simplified feeder and launcher motor dynamics.

The simulator does not model collisions, wheel slip, fuel movement, or interaction with field
elements. The estimated pose will therefore track ground truth more closely than it does on carpet.

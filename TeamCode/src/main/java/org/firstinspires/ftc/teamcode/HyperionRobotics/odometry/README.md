# odometry — PinpointOdometry

Wrapper around the goBILDA Pinpoint Odometry Computer for field pose (X, Y, heading).

**Class:** `org.firstinspires.ftc.teamcode.hyperion.odometry.PinpointOdometry`  
**File:** `PinpointOdometry.java`

## Constructor

```java
public PinpointOdometry(HardwareMap hardwareMap)
```

Gets the Pinpoint device, applies X/Y offsets from `RobotConstants`, sets encoder resolution/directions, and resets pose + IMU.

## Methods

| Method | Returns | Description |
|--------|---------|-------------|
| `update()` | `void` | **Call once per loop** before reading pose |
| `reset()` | `void` | Resets pose and IMU (`resetPosAndIMU`) |
| `setPose(double xInches, double yInches, double headingDeg)` | `void` | Seeds / overwrites the current pose |
| `getPose()` | `Pose2D` | Full pose object from Pinpoint |
| `getX()` | `double` | X position in inches |
| `getY()` | `double` | Y position in inches |
| `getHeadingDeg()` | `double` | Heading in degrees |
| `getHeadingRad()` | `double` | Heading in radians |
| `correctPose(double x, double y, double headingDeg)` | `void` | Alias for `setPose` — used when Limelight corrects drift |
| `getDriver()` | `GoBildaPinpointDriver` | Raw Pinpoint device for advanced use |

## How to use

```java
PinpointOdometry odo = new PinpointOdometry(hardwareMap);
odo.setPose(0, 0, 0); // start pose

while (opModeIsActive()) {
    odo.update();
    telemetry.addData("Pose", "x=%.1f y=%.1f h=%.1f",
            odo.getX(), odo.getY(), odo.getHeadingDeg());
}
```

### Correcting drift from Limelight

```java
// usually via RobotLocalization.correctOdometry()
odo.correctPose(visionX, visionY, visionHeadingDeg);
```

### Notes

- Offsets (`PINPOINT_X_OFFSET_MM`, `PINPOINT_Y_OFFSET_MM`) must match your mounting (mm from robot center).
- Always `update()` before `getX` / `getY` / `getHeading*`.
- Coordinate units here are **inches** and **degrees** at the wrapper API.

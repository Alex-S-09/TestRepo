# HyperionRobotics Package Overview

Root package for the goBILDA 6-wheel FTC robot code.

**Package:** `org.firstinspires.ftc.teamcode.hyperion`  
**Path:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hyperion/`

## Code flow

```text
OpMode (TeleOp / Auto)
        │
        ▼
  HyperionRobot          ← constructs & owns all subsystems
        │
        ├── DriveTrain
        ├── PinpointOdometry
        ├── Intake
        ├── ViperArm
        └── Limelight
              ├── AprilTagDetection
              ├── DriveToAprilTag
              ├── GamePieceDetection
              ├── AutoAim
              ├── RobotLocalization  ← can correct Pinpoint
              └── DriverAssistance   ← uses AutoAim / DriveToAprilTag / Localization
```

Typical loop:

1. Construct `HyperionRobot` in `init` / before `waitForStart()`.
2. Each loop call `robot.updateSensors()` (Pinpoint + Limelight).
3. Either run driver sticks **or** an assist module (`AutoAim`, `DriverAssistance`, etc.).
4. On stop, call `robot.stopAll()`.

## Directory map

| Directory | Responsibility | README |
|-----------|----------------|--------|
| [`constants/`](constants/README.md) | Hardware names & tunable values | [README](constants/README.md) |
| [`drive/`](drive/README.md) | 6-wheel tank drivetrain | [README](drive/README.md) |
| [`odometry/`](odometry/README.md) | goBILDA Pinpoint pose | [README](odometry/README.md) |
| [`intake/`](intake/README.md) | Intake motor + 2 servos | [README](intake/README.md) |
| [`viper/`](viper/README.md) | 4-stage Viper arm | [README](viper/README.md) |
| [`limelight/`](limelight/README.md) | Vision & driver assists | [README](limelight/README.md) |
| [`opmodes/`](opmodes/README.md) | TeleOp & Autonomous entry points | [README](opmodes/README.md) |

Root class in this folder:

| File | Role |
|------|------|
| `HyperionRobot.java` | Aggregates every subsystem for OpModes |

## HyperionRobot

### Constructor

```java
public HyperionRobot(HardwareMap hardwareMap)
```

Builds drive, odometry, intake, viper, Limelight, and all Limelight feature modules.

### Methods

| Method | Description |
|--------|-------------|
| `updateSensors()` | Calls `odometry.update()` and `limelight.update()` once per loop |
| `stopAll()` | Stops drive/intake/viper, cancels assists, stops Limelight |

### Public fields

`drive`, `odometry`, `intake`, `viper`, `limelight`, `aprilTags`, `driveToAprilTag`, `gamePieces`, `autoAim`, `localization`, `driverAssist`

### Usage

```java
HyperionRobot robot = new HyperionRobot(hardwareMap);
waitForStart();
while (opModeIsActive()) {
    robot.updateSensors();
    robot.drive.arcadeDrive(-gamepad1.left_stick_y, gamepad1.right_stick_x);
}
robot.stopAll();
```

## Drop-in install

Copy this entire `hyperion` folder into your FTC Robot Controller repo:

```bash
cp -R hyperion \
  /path/to/FtcRobotController/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
```

Match device names in [`constants/RobotConstants.java`](constants/RobotConstants.java) to your Driver Station configuration.

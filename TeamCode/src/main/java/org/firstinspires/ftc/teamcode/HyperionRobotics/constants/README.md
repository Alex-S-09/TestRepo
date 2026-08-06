# constants — RobotConstants

Central place for hardware map names and tunable numbers. Change values here instead of hard-coding them in subsystems.

**Class:** `org.firstinspires.ftc.teamcode.hyperion.constants.RobotConstants`  
**File:** `RobotConstants.java`

This class is a `final` utility with a private constructor — use the `public static final` fields only.

## Field groups

### Drive

| Constant | Purpose |
|----------|---------|
| `LEFT_FRONT`, `LEFT_MIDDLE`, `LEFT_BACK` | Left-side motor config names |
| `RIGHT_FRONT`, `RIGHT_MIDDLE`, `RIGHT_BACK` | Right-side motor config names |
| `DRIVE_TICKS_PER_REV` | Encoder ticks per motor revolution |
| `WHEEL_DIAMETER_IN` | Wheel diameter (inches) |
| `DRIVE_GEAR_RATIO` | Extra gearing after the motor |
| `COUNTS_PER_INCH` | Derived encoder counts per inch |
| `TELEOP_DRIVE_SCALE` / `TELEOP_TURN_SCALE` | TeleOp stick scaling |
| `AUTO_DRIVE_POWER` / `AUTO_TURN_POWER` | Default autonomous powers |

### Intake

| Constant | Purpose |
|----------|---------|
| `INTAKE_MOTOR` | Intake motor config name |
| `INTAKE_SERVO_LEFT` / `INTAKE_SERVO_RIGHT` | Servo config names |
| `INTAKE_IN_POWER` / `INTAKE_OUT_POWER` | Motor powers |
| `INTAKE_SERVO_OPEN` / `INTAKE_SERVO_CLOSED` | Servo positions (0–1) |

### Viper arm

| Constant | Purpose |
|----------|---------|
| `VIPER_MOTOR` / `VIPER_MOTOR_2` | Slide motor names |
| `VIPER_DUAL_MOTOR` | `true` if a second motor is installed |
| `VIPER_STAGE_0` … `VIPER_STAGE_4` | Encoder setpoints for each stage |
| `VIPER_MAX_TICKS` | Soft upper limit |
| `VIPER_POWER` | RUN_TO_POSITION power |
| `VIPER_TOLERANCE` | “At target” encoder window |

### Pinpoint

| Constant | Purpose |
|----------|---------|
| `PINPOINT` | Hardware map name |
| `PINPOINT_X_OFFSET_MM` / `PINPOINT_Y_OFFSET_MM` | Pod offsets from robot center |

### Limelight

| Constant | Purpose |
|----------|---------|
| `LIMELIGHT` | Hardware map name |
| `LL_PIPELINE_APRILTAG` / `GAMEPIECE` / `DRIVER` | Pipeline indices |
| `LL_TX_TOLERANCE_DEG` | Aim lock threshold |
| `LL_TA_MIN` | Minimum target area |
| `LL_AIM_KP` / `LL_RANGE_KP` | Steering / ranging gains |
| `LL_DESIRED_TAG_DISTANCE_IN` | Stop distance for drive-to-tag |
| `LL_MAX_AIM_POWER` / `LL_MAX_DRIVE_POWER` | Power clamps |
| `FIELD_SCORING_*` / `FIELD_HANG_*` / `FIELD_PRESET_*` | Assist waypoint poses (inches) |

## How to use

```java
import org.firstinspires.ftc.teamcode.hyperion.constants.RobotConstants;

hardwareMap.get(DcMotorEx.class, RobotConstants.LEFT_FRONT);
double power = RobotConstants.AUTO_DRIVE_POWER;
```

## Tuning checklist

1. Rename constants to match your Robot Controller config.
2. Measure and set Pinpoint offsets after mounting.
3. Calibrate Viper stage ticks with the arm at each height.
4. Set intake servo open/close on the robot.
5. Align Limelight pipeline indices with the Limelight web UI.
6. Update field preset coordinates for your season layout.

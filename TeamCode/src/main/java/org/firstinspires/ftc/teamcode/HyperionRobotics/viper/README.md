# viper — ViperArm

4-stage Viper slide controlled with encoder setpoints (`STOWED` → `MAX`).

**Class:** `org.firstinspires.ftc.teamcode.hyperion.viper.ViperArm`  
**File:** `ViperArm.java`

## Enum: Stage

| Stage | Constant ticks field |
|-------|----------------------|
| `STOWED` | `VIPER_STAGE_0` |
| `LOW` | `VIPER_STAGE_1` |
| `MID` | `VIPER_STAGE_2` |
| `HIGH` | `VIPER_STAGE_3` |
| `MAX` | `VIPER_STAGE_4` |

## Constructor

```java
public ViperArm(HardwareMap hardwareMap)
```

Configures the primary motor (and optional second motor if `VIPER_DUAL_MOTOR` is true) for `RUN_TO_POSITION`, then moves to `STOWED`.

## Methods

| Method | Returns | Description |
|--------|---------|-------------|
| `setStage(Stage stage)` | `void` | Moves to a named stage setpoint |
| `goToTicks(int ticks)` | `void` | Moves to a raw encoder target (clipped to max) |
| `jog(double power)` | `void` | Nudges target ticks while stick is held; holds position near zero |
| `holdPosition()` | `void` | Locks current encoder position |
| `stop()` | `void` | Sets motor power(s) to 0 |
| `isAtTarget()` | `boolean` | Within `VIPER_TOLERANCE` of target |
| `getCurrentTicks()` | `int` | Primary motor encoder position |
| `getCurrentStage()` | `Stage` | Last commanded stage |
| `nextStage()` | `void` | Advances one stage (clamped at `MAX`) |
| `previousStage()` | `void` | Lowers one stage (clamped at `STOWED`) |

## How to use

### Stage buttons

```java
ViperArm viper = new ViperArm(hardwareMap);

if (gamepad2.dpad_up) viper.nextStage();
if (gamepad2.dpad_down) viper.previousStage();

// or explicit:
viper.setStage(ViperArm.Stage.HIGH);
```

### Wait until raised (Auto)

```java
viper.setStage(ViperArm.Stage.HIGH);
while (opModeIsActive() && !viper.isAtTarget()) {
    idle();
}
```

### Manual jog

```java
viper.jog(-gamepad2.left_stick_y);
```

### Notes

- Calibrate `VIPER_STAGE_*` ticks with the slide at each height.
- Set `VIPER_DUAL_MOTOR = true` and configure `viperMotor2` for dual-drive slides.
- Keep power limited and respect `VIPER_MAX_TICKS` to avoid over-extension.

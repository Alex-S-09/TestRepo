# opmodes — TeleOp & Autonomous

Driver Station entry points that wire `HyperionRobot` to gamepads and auto sequences.

**Package:** `org.firstinspires.ftc.teamcode.hyperion.opmodes`

| File | OpMode name | Type |
|------|-------------|------|
| `HyperionTeleOp.java` | `Hyperion TeleOp` | `@TeleOp` |
| `HyperionAutoDrive.java` | `Hyperion Auto Drive` | `@Autonomous` |

Both appear under group **Hyperion** on the Driver Station.

---

## HyperionTeleOp

Full TeleOp: arcade drive, intake, Viper, and Limelight assists.

### Lifecycle

1. Builds `HyperionRobot`.
2. `waitForStart()`.
3. Each loop: `updateSensors()` → handle assists / drive → intake → viper → telemetry.
4. `stopAll()` on exit.

### Gamepad 1 — drive & assists

| Control | Action |
|---------|--------|
| Left stick Y | Forward / reverse |
| Right stick X | Turn |
| A | Start `AutoAim` |
| B | Align scoring (`DriverAssistance.alignScoring`) |
| X | Align hang bar |
| Y | Drive to preset location |
| DPAD left | Keep square to field |
| BACK | Cancel assist |

While an assist is active, stick drive is suppressed and `autoAim.update()` or `driverAssist.update()` runs instead.

### Gamepad 2 — mechanisms

| Control | Action |
|---------|--------|
| A | Intake in |
| B | Outtake |
| X | Intake stop |
| LB / RB | Claw open / close |
| DPAD up / down | Viper next / previous stage |
| Left stick Y | Viper jog |

### Methods

| Method | Description |
|--------|-------------|
| `runOpMode()` | Standard `LinearOpMode` entry |

### Usage

Select **Hyperion TeleOp** on the Driver Station. No code changes required unless you remap buttons.

---

## HyperionAutoDrive

Sample autonomous sequence (tune for your alliance start).

### Sequence

1. Seed Pinpoint pose `(0, 0, 0)`.
2. Drive forward 24 in.
3. Attempt Limelight → Pinpoint correction (`localization.correctOdometry()`).
4. Drive toward nearest AprilTag (up to ~4 s).
5. Raise Viper to `HIGH`, open claw, outtake, close, stow.
6. Drive forward 12 in (park stub).
7. `stopAll()`.

### Methods

| Method | Description |
|--------|-------------|
| `runOpMode()` | Runs the sequence above |
| `driveBlocking(double inches, double power)` | Private helper: `driveInches` + wait until not busy |

### Usage

```text
Driver Station → Autonomous → "Hyperion Auto Drive"
```

Replace distances, tag targeting, and score logic for your starting tile and season tasks. Use `robot.driveToAprilTag.setTargetId(id)` when you need a specific backdrop/substation tag.

### Notes

- Keep `opModeIsActive()` checks around waits.
- Call `robot.updateSensors()` during long loops so Pinpoint/Limelight stay fresh.
- This is a template — validate on the field before competition.

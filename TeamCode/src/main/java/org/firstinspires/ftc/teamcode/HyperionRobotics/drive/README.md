# drive — DriveTrain

6-wheel tank drivetrain (3 motors per side) for TeleOp arcade/tank control and encoder-based Autonomous moves.

**Class:** `org.firstinspires.ftc.teamcode.hyperion.drive.DriveTrain`  
**File:** `DriveTrain.java`

## Constructor

```java
public DriveTrain(HardwareMap hardwareMap)
```

Loads six `DcMotorEx` devices from `RobotConstants`, reverses the left side, sets brake mode, and starts in `RUN_WITHOUT_ENCODER`.

## Methods

| Method | Returns | Description |
|--------|---------|-------------|
| `arcadeDrive(double drive, double turn)` | `void` | Single-stick style: forward + turn, normalized to ±1 |
| `tankDrive(double leftPower, double rightPower)` | `void` | Independent left/right powers (clipped) |
| `setTankPowers(double left, double right)` | `void` | Applies the same power to all three motors per side |
| `stop()` | `void` | Zero all drive motors |
| `driveInches(double inches, double power)` | `void` | Starts `RUN_TO_POSITION` for a straight distance |
| `turnDegreesApprox(double degrees, double power)` | `void` | Approximate in-place tank turn (tune wheelbase) |
| `isBusy()` | `boolean` | `true` while front encoders are still seeking target |
| `finishMotion()` | `void` | Stops motors and returns to `RUN_WITHOUT_ENCODER` |
| `setMode(DcMotor.RunMode mode)` | `void` | Sets mode on all six motors |
| `setZeroPowerBehavior(...)` | `void` | Sets brake/float on all six motors |
| `getLeftEncoder()` | `int` | Left-front encoder ticks |
| `getRightEncoder()` | `int` | Right-front encoder ticks |

## How to use

### TeleOp arcade

```java
DriveTrain drive = new DriveTrain(hardwareMap);
// each loop:
drive.arcadeDrive(-gamepad1.left_stick_y, gamepad1.right_stick_x);
```

### Autonomous straight move

```java
drive.driveInches(24.0, RobotConstants.AUTO_DRIVE_POWER);
while (opModeIsActive() && drive.isBusy()) {
    idle();
}
drive.finishMotion();
```

### Notes

- Middle motors are powered with the same side power (true 6WD).
- If the robot drives backward, flip left/right `Direction` in the constructor.
- Prefer Pinpoint / Limelight for precise heading turns; `turnDegreesApprox` is a rough fallback.

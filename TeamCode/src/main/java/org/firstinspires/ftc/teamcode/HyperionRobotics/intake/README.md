# intake — Intake

Intake motor plus left/right servos (claw or flap pair).

**Class:** `org.firstinspires.ftc.teamcode.hyperion.intake.Intake`  
**File:** `Intake.java`

## Constructor

```java
public Intake(HardwareMap hardwareMap)
```

Loads the motor and both servos, enables brake on the motor, reverses the right servo (mirrored pair), closes the claw, and stops the motor.

## Methods

| Method | Returns | Description |
|--------|---------|-------------|
| `intake()` | `void` | Runs motor at `INTAKE_IN_POWER` |
| `outtake()` | `void` | Runs motor at `INTAKE_OUT_POWER` |
| `stop()` | `void` | Sets motor power to 0 |
| `setPower(double power)` | `void` | Direct motor power (−1…1) |
| `open()` | `void` | Both servos to `INTAKE_SERVO_OPEN` |
| `close()` | `void` | Both servos to `INTAKE_SERVO_CLOSED` |
| `setServoPositions(double left, double right)` | `void` | Manual left/right servo positions |
| `isOpen()` | `boolean` | `true` if left servo is near the open position |

## How to use

### TeleOp buttons

```java
Intake intake = new Intake(hardwareMap);

if (gamepad2.a) intake.intake();
else if (gamepad2.b) intake.outtake();
else if (gamepad2.x) intake.stop();

if (gamepad2.left_bumper) intake.open();
else if (gamepad2.right_bumper) intake.close();
```

### Autonomous score stub

```java
intake.open();
sleep(300);
intake.outtake();
sleep(500);
intake.stop();
intake.close();
```

### Notes

- Tune open/close positions in `RobotConstants` on the real robot.
- If both servos move the same way instead of mirroring, remove `servoRight.setDirection(REVERSE)`.
- `GamePieceDetection.centerOnPiece(true)` can open and run intake when centered on a piece.

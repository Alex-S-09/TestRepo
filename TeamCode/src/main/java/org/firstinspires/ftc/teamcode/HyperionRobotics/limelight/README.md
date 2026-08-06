# limelight — Vision & Driver Assistance

Limelight 3A wrapper plus feature modules for AprilTags, game pieces, auto-aim, localization, and TeleOp assists.

**Package:** `org.firstinspires.ftc.teamcode.hyperion.limelight`

## Module map

| File | Class | Role |
|------|-------|------|
| `Limelight.java` | `Limelight` | Hardware poll / pipeline / tx-ty-ta |
| `AprilTagDetection.java` | `AprilTagDetection` | Tag ID list, nearest tag, distance estimate |
| `DriveToAprilTag.java` | `DriveToAprilTag` | Drive + steer until at tag |
| `GamePieceDetection.java` | `GamePieceDetection` | Color/detector piece find & center |
| `AutoAim.java` | `AutoAim` | One-button rotate-to-target |
| `RobotLocalization.java` | `RobotLocalization` | Field pose from botpose; correct Pinpoint |
| `DriverAssistance.java` | `DriverAssistance` | Scoring / hang / preset / square assists |

Pipeline indices come from `RobotConstants` (`LL_PIPELINE_APRILTAG`, `GAMEPIECE`, `DRIVER`).

---

## Limelight

Shared camera device. Other modules take this in their constructors.

### Methods

| Method | Returns | Description |
|--------|---------|-------------|
| `start()` / `stop()` | `void` | Start or stop Limelight polling |
| `setPipeline(int index)` | `void` | Switch active pipeline |
| `update()` | `LLResult` | Poll and cache latest result — call each loop |
| `getLatest()` | `LLResult` | Cached result |
| `hasTarget()` | `boolean` | Valid target present |
| `getTx()` / `getTy()` / `getTa()` | `double` | Crosshair offset X/Y and area |
| `getDevice()` | `Limelight3A` | Raw FTC Limelight device |

### Usage

```java
Limelight ll = new Limelight(hardwareMap);
ll.setPipeline(RobotConstants.LL_PIPELINE_APRILTAG);
LLResult result = ll.update();
if (ll.hasTarget()) {
    telemetry.addData("tx", ll.getTx());
}
```

---

## AprilTagDetection

Reads fiducial results from the latest Limelight frame.

### Nested type: `TagInfo`

Fields: `id`, `tx`, `ty`, `ta`, `distanceInches`

### Methods

| Method | Returns | Description |
|--------|---------|-------------|
| `useAprilTagPipeline()` | `void` | Switches to AprilTag pipeline |
| `getVisibleTags()` | `List<TagInfo>` | All tags in the current frame |
| `getNearestTag()` | `TagInfo` | Closest by estimated distance (`null` if none) |
| `getTagById(int id)` | `TagInfo` | Specific tag or `null` |
| `seesTag(int id)` | `boolean` | Whether that ID is visible |

### Usage

```java
AprilTagDetection tags = new AprilTagDetection(ll);
ll.update();
AprilTagDetection.TagInfo nearest = tags.getNearestTag();
if (nearest != null) {
    telemetry.addData("Tag", "#%d @ %.1f in", nearest.id, nearest.distanceInches);
}
```

---

## DriveToAprilTag

Closed-loop drive toward a tag using `tx` (steer) and estimated range (forward).

### Methods

| Method | Returns | Description |
|--------|---------|-------------|
| `setTargetId(int id)` | `void` | Chase a specific AprilTag ID |
| `clearTarget()` | `void` | Clear ID; `update()` then uses nearest tag |
| `isArrived()` | `boolean` | Last update reached alignment + range |
| `update()` | `boolean` | Run one control step; `true` when arrived |

### Usage

```java
DriveToAprilTag toTag = new DriveToAprilTag(ll, tags, drive);
toTag.setTargetId(5);
while (opModeIsActive() && !toTag.update()) {
    // waits until aligned and in range
}
```

---

## GamePieceDetection

Uses the game-piece pipeline (color or detector) to find and center on a piece.

### Methods

| Method | Returns | Description |
|--------|---------|-------------|
| `useGamePiecePipeline()` | `void` | Select game-piece pipeline |
| `isPiecePresent()` | `boolean` | Valid target with area ≥ `LL_TA_MIN` |
| `getPieceTx()` / `getPieceTa()` | `double` | Offset / area of current piece |
| `centerOnPiece(boolean runIntake)` | `boolean` | Steer (and optionally intake); `true` when centered |

### Usage

```java
GamePieceDetection pieces = new GamePieceDetection(ll, drive, intake);
while (opModeIsActive() && !pieces.centerOnPiece(true)) {
    idle();
}
```

---

## AutoAim

One-button rotate until `tx` is within `LL_TX_TOLERANCE_DEG`, then stop.

### Methods

| Method | Returns | Description |
|--------|---------|-------------|
| `start()` | `void` | Begin aiming (AprilTag pipeline) |
| `cancel()` | `void` | Abort and stop drive |
| `isActive()` | `boolean` | Aiming in progress |
| `isLocked()` | `boolean` | Successfully locked on last run |
| `update()` | `boolean` | One aim step; `true` when locked |

### Usage

```java
AutoAim aim = new AutoAim(ll, drive);
if (gamepad1.a) aim.start();
if (aim.isActive()) {
    aim.update(); // do not also arcadeDrive this loop
}
```

---

## RobotLocalization

Estimates robot field pose from Limelight botpose (MegaTag) and can push that pose into Pinpoint.

### Nested type: `FieldPose`

Fields: `xInches`, `yInches`, `headingDeg`, `valid`  
Factory: `FieldPose.invalid()`

### Methods

| Method | Returns | Description |
|--------|---------|-------------|
| `useAprilTagPipeline()` | `void` | Select AprilTag / MegaTag pipeline |
| `update()` | `FieldPose` | Read botpose → inches/degrees |
| `getLastPose()` | `FieldPose` | Cached last vision pose |
| `correctOdometry()` | `boolean` | If vision valid, overwrite Pinpoint pose |
| `getBestPose()` | `FieldPose` | Vision if valid, else Pinpoint |

### Usage

```java
RobotLocalization loc = new RobotLocalization(ll, odometry);
if (loc.correctOdometry()) {
    telemetry.addLine("Pinpoint corrected from Limelight");
}
RobotLocalization.FieldPose pose = loc.getBestPose();
```

Requires a field AprilTag map configured on the Limelight.

---

## DriverAssistance

TeleOp helpers that combine the modules above.

### Enum: `Mode`

`IDLE`, `ALIGN_SCORING`, `ALIGN_HANG`, `DRIVE_PRESET`, `SQUARE_FIELD`

### Methods

| Method | Returns | Description |
|--------|---------|-------------|
| `getMode()` | `Mode` | Active assist mode |
| `cancel()` | `void` | Return to idle; stop drive / nested assists |
| `alignScoring(Integer aprilTagId)` | `void` | Align to tag (`null` → AutoAim only) |
| `alignHang()` | `void` | Drive toward hang field pose |
| `driveToPreset()` | `void` | Drive toward preset field pose |
| `keepSquare()` | `void` | Rotate to nearest 90° heading |
| `update()` | `boolean` | Advance active mode; `true` when done / idle |

### Usage

```java
DriverAssistance assist = robot.driverAssist;

if (gamepad1.b) assist.alignScoring(null);
if (gamepad1.x) assist.alignHang();
if (gamepad1.y) assist.driveToPreset();
if (gamepad1.dpad_left) assist.keepSquare();
if (gamepad1.back) assist.cancel();

if (assist.getMode() != DriverAssistance.Mode.IDLE) {
    assist.update(); // replaces manual drive while active
}
```

Preset / hang coordinates are `FIELD_*` values in `RobotConstants`.

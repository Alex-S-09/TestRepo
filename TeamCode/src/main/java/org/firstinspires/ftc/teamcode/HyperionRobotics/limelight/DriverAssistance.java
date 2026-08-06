package org.firstinspires.ftc.teamcode.HyperionRobotics.limelight;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HyperionRobotics.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.HyperionRobotics.drive.DriveTrain;
import org.firstinspires.ftc.teamcode.HyperionRobotics.odometry.PinpointOdometry;

/**
 * TeleOp driver assistance:
 * - Align with scoring locations (AprilTag)
 * - Line up with hanging bar
 * - Drive to a preset field location
 * - Keep the robot square to the field
 */
public class DriverAssistance {
    public enum Mode {
        IDLE,
        ALIGN_SCORING,
        ALIGN_HANG,
        DRIVE_PRESET,
        SQUARE_FIELD
    }

    private final Limelight limelight;
    private final DriveTrain drive;
    private final PinpointOdometry odometry;
    private final AutoAim autoAim;
    private final DriveToAprilTag driveToAprilTag;
    private final RobotLocalization localization;

    private Mode mode = Mode.IDLE;
    private Integer scoringTagId = null;

    public DriverAssistance(
            Limelight limelight,
            DriveTrain drive,
            PinpointOdometry odometry,
            AutoAim autoAim,
            DriveToAprilTag driveToAprilTag,
            RobotLocalization localization) {
        this.limelight = limelight;
        this.drive = drive;
        this.odometry = odometry;
        this.autoAim = autoAim;
        this.driveToAprilTag = driveToAprilTag;
        this.localization = localization;
    }

    public Mode getMode() {
        return mode;
    }

    public void cancel() {
        mode = Mode.IDLE;
        autoAim.cancel();
        driveToAprilTag.clearTarget();
        drive.stop();
    }

    public void alignScoring(Integer aprilTagId) {
        mode = Mode.ALIGN_SCORING;
        scoringTagId = aprilTagId;
        limelight.setPipeline(RobotConstants.LL_PIPELINE_APRILTAG);
        if (aprilTagId != null) {
            driveToAprilTag.setTargetId(aprilTagId);
        } else {
            autoAim.start();
        }
    }

    public void alignHang() {
        mode = Mode.ALIGN_HANG;
        // Hang uses field pose (Pinpoint / Limelight), not necessarily a tag.
    }

    public void driveToPreset() {
        mode = Mode.DRIVE_PRESET;
    }

    public void keepSquare() {
        mode = Mode.SQUARE_FIELD;
    }

    /**
     * Run the active assistance mode. Returns true when the current goal is done.
     */
    public boolean update() {
        if (mode == Mode.IDLE) {
            return true;
        }

        odometry.update();
        limelight.update();

        switch (mode) {
            case ALIGN_SCORING:
                return updateAlignScoring();
            case ALIGN_HANG:
                return driveToFieldPoint(
                        RobotConstants.FIELD_HANG_X,
                        RobotConstants.FIELD_HANG_Y,
                        0.0);
            case DRIVE_PRESET:
                return driveToFieldPoint(
                        RobotConstants.FIELD_PRESET_X,
                        RobotConstants.FIELD_PRESET_Y,
                        0.0);
            case SQUARE_FIELD:
                return squareToField();
            default:
                return true;
        }
    }

    private boolean updateAlignScoring() {
        if (scoringTagId != null) {
            boolean done = driveToAprilTag.update();
            if (done) {
                mode = Mode.IDLE;
            }
            return done;
        }
        boolean done = autoAim.update();
        if (done) {
            mode = Mode.IDLE;
        }
        return done;
    }

    private boolean driveToFieldPoint(double targetX, double targetY, double targetHeadingDeg) {
        RobotLocalization.FieldPose pose = localization.getBestPose();
        double dx = targetX - pose.xInches;
        double dy = targetY - pose.yInches;
        double distance = Math.hypot(dx, dy);
        double fieldBearing = Math.toDegrees(Math.atan2(dy, dx));
        double headingError = wrapDeg(fieldBearing - pose.headingDeg);

        if (distance < 2.0 && Math.abs(wrapDeg(targetHeadingDeg - pose.headingDeg)) < 3.0) {
            drive.stop();
            mode = Mode.IDLE;
            return true;
        }

        double forward = Range.clip(distance * 0.03, -RobotConstants.LL_MAX_DRIVE_POWER, RobotConstants.LL_MAX_DRIVE_POWER);
        // Only drive forward when roughly facing the target
        if (Math.abs(headingError) > 25.0) {
            forward = 0.0;
        }
        double turn = Range.clip(
                headingError * RobotConstants.LL_AIM_KP,
                -RobotConstants.LL_MAX_AIM_POWER,
                RobotConstants.LL_MAX_AIM_POWER);
        drive.arcadeDrive(forward, turn);
        return false;
    }

    private boolean squareToField() {
        odometry.update();
        double heading = odometry.getHeadingDeg();
        // Snap to nearest 90°
        double target = Math.round(heading / 90.0) * 90.0;
        double error = wrapDeg(target - heading);
        if (Math.abs(error) < 2.0) {
            drive.stop();
            mode = Mode.IDLE;
            return true;
        }
        double turn = Range.clip(
                error * RobotConstants.LL_AIM_KP,
                -RobotConstants.LL_MAX_AIM_POWER,
                RobotConstants.LL_MAX_AIM_POWER);
        drive.arcadeDrive(0.0, turn);
        return false;
    }

    private static double wrapDeg(double deg) {
        while (deg > 180.0) deg -= 360.0;
        while (deg < -180.0) deg += 360.0;
        return deg;
    }
}

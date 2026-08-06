package org.firstinspires.ftc.teamcode.HyperionRobotics.limelight;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HyperionRobotics.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.HyperionRobotics.drive.DriveTrain;

/**
 * Drive toward a chosen AprilTag using Limelight tx / distance feedback.
 */
public class DriveToAprilTag {
    private final Limelight limelight;
    private final AprilTagDetection aprilTags;
    private final DriveTrain drive;

    private Integer targetId = null;
    private boolean arrived = false;

    public DriveToAprilTag(Limelight limelight, AprilTagDetection aprilTags, DriveTrain drive) {
        this.limelight = limelight;
        this.aprilTags = aprilTags;
        this.drive = drive;
    }

    public void setTargetId(int id) {
        targetId = id;
        arrived = false;
    }

    public void clearTarget() {
        targetId = null;
        arrived = false;
    }

    public boolean isArrived() {
        return arrived;
    }

    /**
     * Call each loop while assisted drive is active.
     * @return true when aligned and within desired range
     */
    public boolean update() {
        aprilTags.useAprilTagPipeline();
        limelight.update();

        AprilTagDetection.TagInfo tag = (targetId == null)
                ? aprilTags.getNearestTag()
                : aprilTags.getTagById(targetId);

        if (tag == null) {
            drive.stop();
            return false;
        }

        double steer = Range.clip(
                -tag.tx * RobotConstants.LL_AIM_KP,
                -RobotConstants.LL_MAX_AIM_POWER,
                RobotConstants.LL_MAX_AIM_POWER);

        double rangeError = tag.distanceInches - RobotConstants.LL_DESIRED_TAG_DISTANCE_IN;
        double drivePower = Range.clip(
                rangeError * RobotConstants.LL_RANGE_KP,
                -RobotConstants.LL_MAX_DRIVE_POWER,
                RobotConstants.LL_MAX_DRIVE_POWER);

        boolean aligned = Math.abs(tag.tx) < RobotConstants.LL_TX_TOLERANCE_DEG;
        boolean inRange = Math.abs(rangeError) < 1.5;

        if (aligned && inRange) {
            drive.stop();
            arrived = true;
            return true;
        }

        drive.arcadeDrive(drivePower, steer);
        arrived = false;
        return false;
    }
}

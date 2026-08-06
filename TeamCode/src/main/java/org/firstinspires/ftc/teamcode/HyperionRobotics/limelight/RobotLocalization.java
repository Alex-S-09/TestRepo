package org.firstinspires.ftc.teamcode.HyperionRobotics.limelight;

import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.HyperionRobotics.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.HyperionRobotics.odometry.PinpointOdometry;

/**
 * Full-field localization from Limelight MegaTag / botpose, with optional
 * Pinpoint drift correction.
 */
public class RobotLocalization {
    public static class FieldPose {
        public final double xInches;
        public final double yInches;
        public final double headingDeg;
        public final boolean valid;

        public FieldPose(double xInches, double yInches, double headingDeg, boolean valid) {
            this.xInches = xInches;
            this.yInches = yInches;
            this.headingDeg = headingDeg;
            this.valid = valid;
        }

        public static FieldPose invalid() {
            return new FieldPose(0, 0, 0, false);
        }
    }

    private final Limelight limelight;
    private final PinpointOdometry odometry;
    private FieldPose lastPose = FieldPose.invalid();

    public RobotLocalization(Limelight limelight, PinpointOdometry odometry) {
        this.limelight = limelight;
        this.odometry = odometry;
    }

    public void useAprilTagPipeline() {
        limelight.setPipeline(RobotConstants.LL_PIPELINE_APRILTAG);
    }

    /**
     * Read robot field pose from Limelight botpose (meters → inches).
     * Requires AprilTag field map configured on the Limelight.
     */
    public FieldPose update() {
        useAprilTagPipeline();
        LLResult result = limelight.update();
        if (result == null || !result.isValid()) {
            lastPose = FieldPose.invalid();
            return lastPose;
        }

        Pose3D botpose = result.getBotpose();
        if (botpose == null) {
            lastPose = FieldPose.invalid();
            return lastPose;
        }

        double xIn = botpose.getPosition().x * 39.3701;
        double yIn = botpose.getPosition().y * 39.3701;
        double yawDeg = botpose.getOrientation().getYaw(AngleUnit.DEGREES);

        lastPose = new FieldPose(xIn, yIn, yawDeg, true);
        return lastPose;
    }

    public FieldPose getLastPose() {
        return lastPose;
    }

    /** Push Limelight pose into Pinpoint to correct odometry drift. */
    public boolean correctOdometry() {
        FieldPose pose = update();
        if (!pose.valid) {
            return false;
        }
        odometry.correctPose(pose.xInches, pose.yInches, pose.headingDeg);
        return true;
    }

    /** Prefer Limelight when valid; otherwise fall back to Pinpoint. */
    public FieldPose getBestPose() {
        FieldPose vision = update();
        if (vision.valid) {
            return vision;
        }
        odometry.update();
        return new FieldPose(odometry.getX(), odometry.getY(), odometry.getHeadingDeg(), true);
    }
}

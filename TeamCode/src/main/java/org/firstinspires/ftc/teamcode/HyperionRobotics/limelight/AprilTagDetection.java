package org.firstinspires.ftc.teamcode.HyperionRobotics.limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.HyperionRobotics.constants.RobotConstants;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * AprilTag detection helpers: ID, angles, area, and distance.
 */
public class AprilTagDetection {

    public static class TagInfo {
        public final int id;
        public final double tx;
        public final double ty;
        public final double ta;
        public final double distanceInches;

        // Raw pose coordinates, useful for telemetry and debugging.
        public final double poseXInches;
        public final double poseYInches;
        public final double poseZInches;

        public TagInfo(
                int id,
                double tx,
                double ty,
                double ta,
                double distanceInches,
                double poseXInches,
                double poseYInches,
                double poseZInches
        ) {
            this.id = id;
            this.tx = tx;
            this.ty = ty;
            this.ta = ta;
            this.distanceInches = distanceInches;
            this.poseXInches = poseXInches;
            this.poseYInches = poseYInches;
            this.poseZInches = poseZInches;
        }
    }

    private static final double MIN_VALID_DISTANCE_IN = 2.0;
    private static final double MAX_VALID_DISTANCE_IN = 300.0;
    private static final double AREA_DISTANCE_SCALE = 38.02;

    private final Limelight limelight;

    public AprilTagDetection(Limelight limelight) {
        this.limelight = limelight;
    }

    public void useAprilTagPipeline() {
        limelight.setPipeline(
                RobotConstants.LL_PIPELINE_APRILTAG
        );
    }

    public List<TagInfo> getVisibleTags() {
        LLResult result = limelight.getLatest();

        if (result == null || !result.isValid()) {
            return Collections.emptyList();
        }

        List<LLResultTypes.FiducialResult> fiducials =
                result.getFiducialResults();

        if (fiducials == null || fiducials.isEmpty()) {
            return Collections.emptyList();
        }

        List<TagInfo> tags = new ArrayList<>();

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            /*
             * Keep the raw pose only for diagnostics. Do not use it
             * for driving because this Limelight is returning a pose
             * approximately ten times larger than the real distance.
             */
            Position robotPosition = fiducial
                    .getRobotPoseTargetSpace()
                    .getPosition()
                    .toUnit(DistanceUnit.INCH);

            double x = robotPosition.x;
            double y = robotPosition.y;
            double z = robotPosition.z;

            // Fiducial area is normalized 0–1; convert it to percentage 0–100.
            double targetArea = fiducial.getTargetArea() * 100;
            double distanceInches = Double.NaN;

            /*
             * Calibrated using:
             * 72 inches -> area 0.2983
             * 24 inches -> area 2.5095
             */
            if (Double.isFinite(targetArea)
                    && targetArea > 0.01) {
                distanceInches =
                        AREA_DISTANCE_SCALE
                                / Math.sqrt(targetArea);
            }

            /*
             * Always preserve the detection, even if its distance
             * cannot be calculated.
             */
            tags.add(new TagInfo(
                    fiducial.getFiducialId(),
                    fiducial.getTargetXDegrees(),
                    fiducial.getTargetYDegrees(),
                    targetArea,
                    distanceInches,
                    x,
                    y,
                    z
            ));
        }

        return tags;
    }

    public TagInfo getNearestTag() {
        List<TagInfo> tags = getVisibleTags();

        if (tags.isEmpty()) {
            return null;
        }

        tags.sort(
                Comparator.comparingDouble(
                        tag -> tag.distanceInches
                )
        );

        return tags.get(0);
    }

    public TagInfo getTagById(int id) {
        for (TagInfo tag : getVisibleTags()) {
            if (tag.id == id) {
                return tag;
            }
        }

        return null;
    }

    public boolean seesTag(int id) {
        return getTagById(id) != null;
    }
}
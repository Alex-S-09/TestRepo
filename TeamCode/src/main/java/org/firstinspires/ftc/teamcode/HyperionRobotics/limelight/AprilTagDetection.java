package org.firstinspires.ftc.teamcode.HyperionRobotics.limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.HyperionRobotics.constants.RobotConstants;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * AprilTag detection helpers: ID, angles, area, and nearest-tag selection.
 */
public class AprilTagDetection {
    public static class TagInfo {
        public final int id;
        public final double tx;
        public final double ty;
        public final double ta;
        public final double distanceInches;

        public TagInfo(int id, double tx, double ty, double ta, double distanceInches) {
            this.id = id;
            this.tx = tx;
            this.ty = ty;
            this.ta = ta;
            this.distanceInches = distanceInches;
        }
    }

    private final Limelight limelight;

    public AprilTagDetection(Limelight limelight) {
        this.limelight = limelight;
    }

    public void useAprilTagPipeline() {
        limelight.setPipeline(RobotConstants.LL_PIPELINE_APRILTAG);
    }

    public List<TagInfo> getVisibleTags() {
        LLResult result = limelight.getLatest();
        if (result == null || !result.isValid()) {
            return Collections.emptyList();
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return Collections.emptyList();
        }

        List<TagInfo> tags = new ArrayList<>();
        for (LLResultTypes.FiducialResult f : fiducials) {
            double dist = estimateDistanceInches(f.getTargetArea());
            tags.add(new TagInfo(
                    f.getFiducialId(),
                    f.getTargetXDegrees(),
                    f.getTargetYDegrees(),
                    f.getTargetArea(),
                    dist));
        }
        return tags;
    }

    public TagInfo getNearestTag() {
        List<TagInfo> tags = getVisibleTags();
        if (tags.isEmpty()) {
            return null;
        }
        tags.sort(Comparator.comparingDouble(t -> t.distanceInches));
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

    /** Rough distance from target area. Tune on your field. */
    private double estimateDistanceInches(double ta) {
        if (ta <= 0.01) {
            return Double.POSITIVE_INFINITY;
        }
        return 60.0 / Math.sqrt(ta);
    }
}

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
 * AprilTag detection helpers: ID, angles, area, and calibrated distance.
 *
 * Camera calibration taken after the Limelight was remounted higher / more level:
 *
 *   Distance     TX       TY       TA
 *   24 in       +3.69    +2.65    2.660 %
 *   48 in       +1.28    +0.92    0.665 %
 *   72 in       +1.82    +0.31    0.291 %
 *   96 in       -0.25    -0.12    0.163 %
 *
 * For an AprilTag of fixed physical size, apparent image area is approximately
 * proportional to 1 / distance^2.  The four measurements give:
 *
 *   distance * sqrt(TA) = 39.143, 39.143, 38.840, 38.758
 *
 * Mean scale = 38.971.
 *
 * Therefore:
 *
 *   distanceInches = 38.971 / sqrt(TA_percent)
 */
public class AprilTagDetection {

    public static class TagInfo {
        public final int id;
        public final double tx;
        public final double ty;
        public final double ta;
        public final double distanceInches;

        // Raw pose coordinates are retained only for telemetry/debugging.
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

    // Recalibrated from the four measurements above.
    private static final double AREA_DISTANCE_SCALE = 38.971;

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

        List<LLResultTypes.FiducialResult> fiducials =
                result.getFiducialResults();

        if (fiducials == null || fiducials.isEmpty()) {
            return Collections.emptyList();
        }

        List<TagInfo> tags = new ArrayList<>();

        for (LLResultTypes.FiducialResult fiducial : fiducials) {

            double x = Double.NaN;
            double y = Double.NaN;
            double z = Double.NaN;

            try {
                Position robotPosition = fiducial
                        .getRobotPoseTargetSpace()
                        .getPosition()
                        .toUnit(DistanceUnit.INCH);

                x = robotPosition.x;
                y = robotPosition.y;
                z = robotPosition.z;
            } catch (Exception ignored) {
                // Pose is diagnostic only. Preserve the tag even if pose is absent.
            }

            // FTC Limelight fiducial area is normalized 0-1. Convert to percent.
            double targetArea = fiducial.getTargetArea() * 100.0;

            double distanceInches = Double.NaN;

            if (Double.isFinite(targetArea) && targetArea > 0.01) {
                double calculated =
                        AREA_DISTANCE_SCALE / Math.sqrt(targetArea);

                if (calculated >= MIN_VALID_DISTANCE_IN
                        && calculated <= MAX_VALID_DISTANCE_IN) {
                    distanceInches = calculated;
                }
            }

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
        List<TagInfo> tags = new ArrayList<>(getVisibleTags());

        if (tags.isEmpty()) {
            return null;
        }

        tags.sort(
                Comparator.comparingDouble(
                        tag -> Double.isFinite(tag.distanceInches)
                                ? tag.distanceInches
                                : Double.POSITIVE_INFINITY
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
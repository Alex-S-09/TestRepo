package org.firstinspires.ftc.teamcode.HyperionRobotics.limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.HyperionRobotics.constants.RobotConstants;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Unified Limelight 3A abstraction for Hyperion Robotics.
 *
 * Current pipeline plan:
 *
 *   Pipeline 0 = AprilTag
 *   Pipeline 1 = Color / yellow-ball tracking
 *   Pipeline 2 = Driver / spare
 *
 * Call update() once per robot control loop before reading results.
 */
public class SensorLimelight3A {

    // =====================================================================
    // Public result types
    // =====================================================================

    public static class ColorTarget {
        public final double tx;
        public final double ty;
        public final double area;

        public ColorTarget(
                double tx,
                double ty,
                double area
        ) {
            this.tx = tx;
            this.ty = ty;
            this.area = area;
        }

        @Override
        public String toString() {
            return String.format(
                    "TX=%.2f TY=%.2f Area=%.4f",
                    tx,
                    ty,
                    area
            );
        }
    }

    public static class DetectorTarget {
        public final String className;
        public final double tx;
        public final double ty;
        public final double area;

        public DetectorTarget(
                String className,
                double tx,
                double ty,
                double area
        ) {
            this.className =
                    className != null
                            ? className
                            : "";

            this.tx = tx;
            this.ty = ty;
            this.area = area;
        }
    }

    public static class AprilTagTarget {
        public final int id;
        public final String family;
        public final double tx;
        public final double ty;
        public final double area;
        public final Pose3D robotPoseTargetSpace;

        public AprilTagTarget(
                int id,
                String family,
                double tx,
                double ty,
                double area,
                Pose3D robotPoseTargetSpace
        ) {
            this.id = id;
            this.family =
                    family != null
                            ? family
                            : "";

            this.tx = tx;
            this.ty = ty;
            this.area = area;
            this.robotPoseTargetSpace =
                    robotPoseTargetSpace;
        }

        /*
         * AprilTag distance calibration from the physical measurements:
         *
         *   24 in -> TA 2.660%
         *   48 in -> TA 0.665%
         *   72 in -> TA 0.291%
         *   96 in -> TA 0.163%
         *
         * distance ~= 38.971 / sqrt(TA_percent)
         *
         * IMPORTANT:
         * "area" is stored as PERCENT in AprilTagTarget, not normalized 0-1.
         */
        public double getDistanceInches() {

            final double AREA_DISTANCE_SCALE = 38.971;

            if (!Double.isFinite(area)
                    || area <= 0.01) {

                return Double.NaN;
            }

            double distance =
                    AREA_DISTANCE_SCALE
                            / Math.sqrt(area);

            /*
             * Reject physically implausible values instead of allowing
             * zero/bad Limelight pose data to look like "already at target".
             */
            if (!Double.isFinite(distance)
                    || distance < 2.0
                    || distance > 300.0) {

                return Double.NaN;
            }

            return distance;
        }
    }

    // =====================================================================
    // Hardware and cached state
    // =====================================================================

    private final Limelight3A limelight;

    private LLResult latestResult;
    private LLStatus latestStatus;

    private int requestedPipeline =
            RobotConstants.LL_PIPELINE_APRILTAG;

    // =====================================================================
    // Construction / lifecycle
    // =====================================================================

    public SensorLimelight3A(
            HardwareMap hardwareMap
    ) {

        limelight =
                hardwareMap.get(
                        Limelight3A.class,
                        RobotConstants.LIMELIGHT
                );

        limelight.setPollRateHz(100);

        useAprilTagPipeline();

        limelight.start();
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    /**
     * Poll once per loop and cache one consistent frame.
     */
    public void update() {

        latestResult =
                limelight.getLatestResult();

        latestStatus =
                limelight.getStatus();
    }

    // =====================================================================
    // Pipeline management
    // =====================================================================

    public void setPipeline(
            int pipelineIndex
    ) {

        requestedPipeline =
                pipelineIndex;

        limelight.pipelineSwitch(
                pipelineIndex
        );
    }

    public void useAprilTagPipeline() {

        setPipeline(
                RobotConstants.LL_PIPELINE_APRILTAG
        );
    }

    public void useGamePiecePipeline() {

        setPipeline(
                RobotConstants.LL_PIPELINE_GAMEPIECE
        );
    }

    public void useDriverPipeline() {

        setPipeline(
                RobotConstants.LL_PIPELINE_DRIVER
        );
    }

    public int getRequestedPipeline() {
        return requestedPipeline;
    }

    public int getActivePipelineIndex() {

        return latestResult != null
                ? latestResult.getPipelineIndex()
                : -1;
    }

    public String getActivePipelineType() {

        if (latestStatus == null
                || latestStatus.getPipelineType() == null) {

            return "unknown";
        }

        return latestStatus
                .getPipelineType()
                .toString();
    }

    // =====================================================================
    // General result information
    // =====================================================================

    public LLResult getRawResult() {
        return latestResult;
    }

    public LLStatus getRawStatus() {
        return latestStatus;
    }

    public boolean hasResult() {
        return latestResult != null;
    }

    public boolean isValid() {

        return latestResult != null
                && latestResult.isValid();
    }

    public long getStalenessMs() {

        return latestResult != null
                ? latestResult.getStaleness()
                : Long.MAX_VALUE;
    }

    public boolean isFresh(
            long maxAgeMs
    ) {

        return isValid()
                && getStalenessMs() <= maxAgeMs;
    }

    public double getTx() {

        return isValid()
                ? latestResult.getTx()
                : Double.NaN;
    }

    public double getTy() {

        return isValid()
                ? latestResult.getTy()
                : Double.NaN;
    }

    public double getTa() {

        return isValid()
                ? latestResult.getTa()
                : Double.NaN;
    }

    public double getCaptureLatencyMs() {

        return latestResult != null
                ? latestResult.getCaptureLatency()
                : Double.NaN;
    }

    public double getTargetingLatencyMs() {

        return latestResult != null
                ? latestResult.getTargetingLatency()
                : Double.NaN;
    }

    public double getParseLatencyMs() {

        return latestResult != null
                ? latestResult.getParseLatency()
                : Double.NaN;
    }

    // =====================================================================
    // Color pipeline
    // =====================================================================

    /**
     * Return every ColorResult from the current frame.
     *
     * No area threshold is applied here.
     */
    public List<ColorTarget> getColorTargets() {

        if (!isValid()) {
            return Collections.emptyList();
        }

        List<LLResultTypes.ColorResult> rawTargets =
                latestResult.getColorResults();

        if (rawTargets == null
                || rawTargets.isEmpty()) {

            return Collections.emptyList();
        }

        List<ColorTarget> targets =
                new ArrayList<>();

        for (LLResultTypes.ColorResult raw : rawTargets) {

            if (raw == null) {
                continue;
            }

            targets.add(
                    new ColorTarget(
                            raw.getTargetXDegrees(),
                            raw.getTargetYDegrees(),
                            raw.getTargetArea()
                    )
            );
        }

        return targets;
    }

    /**
     * Select the color target closest to the camera centerline.
     *
     * This is a good default for the tank robot because it minimizes the
     * steering correction required to reach the selected object.
     */
    public ColorTarget getBestColorTarget() {

        List<ColorTarget> targets =
                getColorTargets();

        if (targets.isEmpty()) {
            return null;
        }

        return Collections.min(
                targets,
                Comparator.comparingDouble(
                        target ->
                                Math.abs(target.tx)
                )
        );
    }

    /**
     * Return largest-area color target.
     *
     * Useful as diagnostic information when comparing multiple yellow blobs.
     */
    public ColorTarget getLargestColorTarget() {

        List<ColorTarget> targets =
                getColorTargets();

        if (targets.isEmpty()) {
            return null;
        }

        return Collections.max(
                targets,
                Comparator.comparingDouble(
                        target ->
                                target.area
                )
        );
    }

    public boolean seesColorTarget() {

        return getBestColorTarget()
                != null;
    }

    /**
     * Game piece currently uses the color pipeline.
     */
    public ColorTarget getBestGamePieceTarget() {

        return getBestColorTarget();
    }

    public boolean seesGamePiece() {

        return getBestGamePieceTarget()
                != null;
    }

    // =====================================================================
    // Neural detector support retained for future use
    // =====================================================================

    public List<DetectorTarget> getDetectorTargets() {

        if (!isValid()) {
            return Collections.emptyList();
        }

        List<LLResultTypes.DetectorResult> rawTargets =
                latestResult.getDetectorResults();

        if (rawTargets == null
                || rawTargets.isEmpty()) {

            return Collections.emptyList();
        }

        List<DetectorTarget> targets =
                new ArrayList<>();

        for (LLResultTypes.DetectorResult raw : rawTargets) {

            if (raw == null) {
                continue;
            }

            targets.add(
                    new DetectorTarget(
                            raw.getClassName(),
                            raw.getTargetXDegrees(),
                            raw.getTargetYDegrees(),
                            raw.getTargetArea()
                    )
            );
        }

        return targets;
    }

    // =====================================================================
    // AprilTag support
    // =====================================================================

    public List<AprilTagTarget> getAprilTags() {

        if (!isValid()) {
            return Collections.emptyList();
        }

        List<LLResultTypes.FiducialResult> rawTags =
                latestResult.getFiducialResults();

        if (rawTags == null
                || rawTags.isEmpty()) {

            return Collections.emptyList();
        }

        List<AprilTagTarget> tags =
                new ArrayList<>();

        for (LLResultTypes.FiducialResult raw : rawTags) {

            if (raw == null) {
                continue;
            }

            tags.add(
                    new AprilTagTarget(
                            raw.getFiducialId(),
                            raw.getFamily(),
                            raw.getTargetXDegrees(),
                            raw.getTargetYDegrees(),

                            /*
                             * FTC Limelight FiducialResult target area is
                             * normalized 0-1.  Store AprilTag area as percent
                             * so it matches the physical calibration above.
                             */
                            raw.getTargetArea() * 100.0,

                            raw.getRobotPoseTargetSpace()
                    )
            );
        }

        return tags;
    }

    public AprilTagTarget getAprilTag(
            int tagId
    ) {

        for (AprilTagTarget tag
                : getAprilTags()) {

            if (tag.id == tagId) {
                return tag;
            }
        }

        return null;
    }

    public boolean seesAprilTag(
            int tagId
    ) {

        return getAprilTag(tagId)
                != null;
    }

    // =====================================================================
    // Limelight hardware status
    // =====================================================================

    public String getName() {

        if (latestStatus == null
                || latestStatus.getName() == null) {

            return "limelight";
        }

        return latestStatus.getName();
    }

    public double getTemperatureC() {

        return latestStatus != null
                ? latestStatus.getTemp()
                : Double.NaN;
    }

    public double getCpuPercent() {

        return latestStatus != null
                ? latestStatus.getCpu()
                : Double.NaN;
    }

    public double getFps() {

        return latestStatus != null
                ? latestStatus.getFps()
                : Double.NaN;
    }

    public Limelight3A getDevice() {
        return limelight;
    }
}
package org.firstinspires.ftc.teamcode.HyperionRobotics.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HyperionRobotics.HyperionRobot;
import org.firstinspires.ftc.teamcode.HyperionRobotics.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.HyperionRobotics.limelight.AprilTagDetection;

import java.util.List;

/**
 * Autonomous sequence:
 * 1. Find AprilTag 12.
 * 2. Approach and stop about 24 inches away.
 * 3. Switch to the yellow game-piece pipeline.
 * 4. Look ahead, sweep both sides, approach a detected ball, and intake it.
 */
@Autonomous(name = "Jarvis Limelight", group = "Autonomous")
public class JarvisLimelight extends LinearOpMode {

    // -------------------------------------------------------------------------
    // AprilTag approach settings
    // -------------------------------------------------------------------------

    private static final int TARGET_TAG_ID = 12;
    private static final double STOP_DISTANCE_IN = 24.0;
    private static final double ARRIVAL_TOLERANCE_IN = 1.0;

    private static final double MIN_VALID_DISTANCE_IN = 2.0;
    private static final double MAX_VALID_DISTANCE_IN = 240.0;
    private static final double MAX_DISTANCE_JUMP_IN = 18.0;
    private static final int REQUIRED_STABLE_TAG_FRAMES = 3;

    private static final double APRILTAG_FAR_POWER = 0.60;
    private static final double APRILTAG_MIDDLE_POWER = 0.30;
    private static final double APRILTAG_NEAR_POWER = 0.16;

    private static final double TAG_SEARCH_POWER = 0.10;
    private static final double LOST_TAG_GRACE_SEC = 0.35;
    private static final double MAX_TAG_APPROACH_TIME_SEC = 15.0;

    /** Flip this if AprilTag steering turns away from the tag. */
    private static final double TAG_STEER_SIGN = 1.0;

    // -------------------------------------------------------------------------
    // Yellow game-piece settings
    // -------------------------------------------------------------------------

    private static final double MAX_COLLECTION_TIME_SEC = 25.0;

    /*
     * Measured background noise was about 0.005 and a real ball was about 0.012
     * at the initial collection distance. Pipeline 1 must still be tuned so
     * non-ball yellow contours are rejected.
     */
    private static final double MIN_BALL_AREA = 0.0075;
    /*
     * With the corrected color pipeline, a ball about 24 inches away reports
     * TA near 0.36. Do not treat that as captured. Start the intake only when
     * the target is substantially larger, and begin the final capture drive
     * when the ball is close to the intake entrance.
     */
    private static final double START_INTAKE_AREA = 1.00;
    private static final double CAPTURE_AREA = 2.50;

    private static final double FORWARD_ALIGNMENT_DEG = 10.0;
    private static final double CAPTURE_ALIGNMENT_DEG = 5.0;
    private static final int REQUIRED_STABLE_BALL_FRAMES = 3;

    private static final double BALL_APPROACH_POWER = 0.25;
    private static final double BALL_PARTIAL_APPROACH_POWER = 0.12;
    private static final double BALL_MAX_STEER_POWER = 0.25;

    /** Flip this if vision steering turns away from the yellow ball. */
    private static final double BALL_STEER_SIGN = 1.0;

    private static final double FINAL_CAPTURE_TIME_SEC = 1.25;
    private static final double FINAL_CAPTURE_POWER = 0.18;

    /*
     * Pause every 30 degrees instead of making one continuous 180-degree
     * sweep. This gives pipeline 1 several sharp, stationary frames in which
     * to recognize a small ball at about two feet.
     */
    private static final double[] BALL_SEARCH_OFFSETS_DEG = {
            0.0,
            30.0,
            60.0,
            90.0,
            60.0,
            30.0,
            0.0,
            -30.0,
            -60.0,
            -90.0,
            -60.0,
            -30.0
    };

    private static final double SEARCH_HEADING_KP = 0.012;
    // These values must overcome drivetrain static friction during the sweep.
    private static final double MIN_SEARCH_TURN_POWER = 0.16;
    private static final double MAX_SEARCH_TURN_POWER = 0.24;
    private static final double SEARCH_HEADING_TOLERANCE_DEG = 10.0;
    private static final double INITIAL_AHEAD_HOLD_SEC = 0.75;
    private static final double SEARCH_WAYPOINT_HOLD_SEC = 0.35;

    /*
     * Pinpoint heading normally increases counterclockwise, while positive
     * arcade turn appears to turn this robot clockwise. Flip this if the
     * displayed heading error grows during search instead of shrinking.
     */
    private static final double ODOMETRY_TURN_SIGN = -1.0;

    private HyperionRobot robot;

    @Override
    public void runOpMode() {
        robot = new HyperionRobot(hardwareMap);

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) {
            return;
        }

        boolean reachedTag = approachAprilTag();
        robot.drive.stop();

        if (!reachedTag || !opModeIsActive()) {
            robot.stopAll();
            telemetry.addData("Status", "AprilTag approach failed or timed out");
            telemetry.update();
            sleep(1500);
            return;
        }

        telemetry.addData(
                "Status",
                "Reached %.1f inches. Searching for yellow ball...",
                STOP_DISTANCE_IN
        );
        telemetry.update();

        collectGamePiece();

        robot.stopAll();
        telemetry.addData("Status", "Autonomous Complete");
        telemetry.update();
        sleep(1000);
    }

    /**
     * Drives toward Tag 12 using the calibrated distance supplied by
     * AprilTagDetection.
     */
    private boolean approachAprilTag() {
        robot.aprilTags.useAprilTagPipeline();

        telemetry.addData("Status", "Switching to AprilTag pipeline...");
        telemetry.update();
        sleep(1000);

        ElapsedTime approachTimer = new ElapsedTime();
        ElapsedTime timeSinceTagSeen = new ElapsedTime();

        boolean hasSeenTarget = false;
        int stableFrameCount = 0;

        double previousMeasuredDistance = Double.NaN;
        double lastKnownDistance = Double.NaN;
        double lastKnownTx = 0.0;

        while (opModeIsActive()
                && approachTimer.seconds() < MAX_TAG_APPROACH_TIME_SEC) {

            robot.updateSensors();

            List<AprilTagDetection.TagInfo> visibleTags =
                    robot.aprilTags.getVisibleTags();

            AprilTagDetection.TagInfo targetTag =
                    findTagById(visibleTags, TARGET_TAG_ID);

            addAprilTagTelemetry(
                    visibleTags,
                    targetTag,
                    approachTimer.seconds()
            );

            if (targetTag == null) {
                stableFrameCount = 0;
                robot.drive.stop();

                if (!hasSeenTarget) {
                    telemetry.addData("Phase", "Waiting for Tag %d", TARGET_TAG_ID);
                } else {
                    double lostTime = timeSinceTagSeen.seconds();
                    telemetry.addData("Lost Time", "%.2f sec", lostTime);

                    if (lostTime <= LOST_TAG_GRACE_SEC) {
                        telemetry.addData("Phase", "Brief dropout - stopped");
                    } else {
                        double searchDirection = Math.abs(lastKnownTx) > 0.5
                                ? Math.signum(TAG_STEER_SIGN * lastKnownTx)
                                : 1.0;

                        robot.drive.arcadeDrive(
                                0.0,
                                searchDirection * TAG_SEARCH_POWER
                        );

                        telemetry.addData("Phase", "Searching for Tag %d", TARGET_TAG_ID);
                    }
                }

                telemetry.update();
                idle();
                continue;
            }

            hasSeenTarget = true;
            timeSinceTagSeen.reset();

            double measuredDistance = targetTag.distanceInches;

            boolean plausibleDistance =
                    Double.isFinite(measuredDistance)
                            && measuredDistance >= MIN_VALID_DISTANCE_IN
                            && measuredDistance <= MAX_VALID_DISTANCE_IN;

            boolean plausibleChange =
                    Double.isNaN(previousMeasuredDistance)
                            || Math.abs(measuredDistance - previousMeasuredDistance)
                            <= MAX_DISTANCE_JUMP_IN;

            if (!plausibleDistance || !plausibleChange) {
                robot.drive.stop();
                stableFrameCount = 0;

                telemetry.addData("Phase", "Rejected unstable measurement");
                telemetry.addData("Rejected Distance", "%.1f in", measuredDistance);

                if (!Double.isNaN(previousMeasuredDistance)) {
                    telemetry.addData(
                            "Previous Distance",
                            "%.1f in",
                            previousMeasuredDistance
                    );
                }

                previousMeasuredDistance = measuredDistance;
                telemetry.update();
                idle();
                continue;
            }

            previousMeasuredDistance = measuredDistance;
            stableFrameCount++;

            if (stableFrameCount < REQUIRED_STABLE_TAG_FRAMES) {
                robot.drive.stop();
                telemetry.addData(
                        "Phase",
                        "Validating distance %d/%d",
                        stableFrameCount,
                        REQUIRED_STABLE_TAG_FRAMES
                );
                telemetry.addData("Candidate Distance", "%.1f in", measuredDistance);
                telemetry.update();
                idle();
                continue;
            }

            lastKnownDistance = measuredDistance;
            lastKnownTx = targetTag.tx;

            if (lastKnownDistance <= STOP_DISTANCE_IN + ARRIVAL_TOLERANCE_IN) {
                robot.drive.stop();
                telemetry.addData("Phase", "Target distance reached");
                telemetry.addData("Distance", "%.1f in", lastKnownDistance);
                telemetry.update();
                return true;
            }

            double rangeError = lastKnownDistance - STOP_DISTANCE_IN;

            double steerPower = Range.clip(
                    TAG_STEER_SIGN * targetTag.tx * RobotConstants.LL_AIM_KP,
                    -RobotConstants.LL_MAX_AIM_POWER,
                    RobotConstants.LL_MAX_AIM_POWER
            );

            double speedLimit;
            if (lastKnownDistance > 48.0) {
                speedLimit = APRILTAG_FAR_POWER;
            } else if (lastKnownDistance > 32.0) {
                speedLimit = APRILTAG_MIDDLE_POWER;
            } else {
                speedLimit = APRILTAG_NEAR_POWER;
            }

            double forwardPower = Range.clip(
                    rangeError * RobotConstants.LL_RANGE_KP,
                    0.0,
                    speedLimit
            );

            robot.drive.arcadeDrive(forwardPower, steerPower);

            telemetry.addData("Phase", "Tracking Tag %d", TARGET_TAG_ID);
            telemetry.addData("Distance", "%.1f in", lastKnownDistance);
            telemetry.addData("Remaining", "%.1f in", rangeError);
            telemetry.addData("TX", "%.2f deg", targetTag.tx);
            telemetry.addData(
                    "Pose XYZ",
                    "%.1f, %.1f, %.1f in",
                    targetTag.poseXInches,
                    targetTag.poseYInches,
                    targetTag.poseZInches
            );
            telemetry.addData("Stable Frames", stableFrameCount);
            telemetry.addData(
                    "Drive / Steer",
                    "%.2f / %.2f",
                    forwardPower,
                    steerPower
            );
            telemetry.update();
            idle();
        }

        robot.drive.stop();
        telemetry.addData("Phase", "AprilTag approach timed out");
        telemetry.update();
        return false;
    }

    /**
     * Looks forward first, sweeps both sides, aligns to a stable yellow target,
     * approaches it, then performs a short final intake movement.
     */
    private void collectGamePiece() {
        robot.gamePieces.useGamePiecePipeline();

        telemetry.addData("Status", "Switching to yellow game-piece pipeline...");
        telemetry.update();
        sleep(500);

        robot.updateSensors();

        double searchCenterHeading = robot.odometry.getHeadingDeg();
        int searchWaypoint = 0;
        int stableTargetFrames = 0;

        boolean captureStarted = false;
        boolean pieceCollected = false;

        ElapsedTime collectionTimer = new ElapsedTime();
        ElapsedTime waypointHoldTimer = new ElapsedTime();
        ElapsedTime captureTimer = new ElapsedTime();

        robot.intake.stop();

        while (opModeIsActive()
                && collectionTimer.seconds() < MAX_COLLECTION_TIME_SEC
                && !pieceCollected) {

            // Poll Limelight and Pinpoint exactly once in each loop.
            robot.updateSensors();

            if (captureStarted) {
                robot.intake.intake();
                robot.drive.arcadeDrive(FINAL_CAPTURE_POWER, 0.0);

                telemetry.addData("Phase", "Final ball intake");
                telemetry.addData(
                        "Capture Time",
                        "%.2f / %.2f sec",
                        captureTimer.seconds(),
                        FINAL_CAPTURE_TIME_SEC
                );

                if (captureTimer.seconds() >= FINAL_CAPTURE_TIME_SEC) {
                    pieceCollected = true;
                    robot.drive.stop();
                }

                telemetry.update();
                idle();
                continue;
            }

            double targetArea = robot.limelight.getTa();
            boolean ballVisible =
                    robot.limelight.hasTarget()
                            && targetArea >= MIN_BALL_AREA;

            if (ballVisible) {
                stableTargetFrames++;
            } else {
                stableTargetFrames = 0;
            }

            telemetry.addData("LL Raw Valid", robot.limelight.hasTarget());
            telemetry.addData("Area Threshold", "%.3f", MIN_BALL_AREA);
            telemetry.addData("Ball Accepted", ballVisible);

            if (ballVisible
                    && stableTargetFrames < REQUIRED_STABLE_BALL_FRAMES) {
                robot.drive.stop();
                robot.intake.stop();
                waypointHoldTimer.reset();

                telemetry.addData("Phase", "Confirming yellow target");
                telemetry.addData(
                        "Target Frames",
                        "%d / %d",
                        stableTargetFrames,
                        REQUIRED_STABLE_BALL_FRAMES
                );
                telemetry.addData("TX", "%.2f deg", robot.limelight.getTx());
                telemetry.addData("Target Area", "%.3f", targetArea);
                telemetry.update();
                idle();
                continue;
            }

            if (ballVisible) {
                waypointHoldTimer.reset();

                double tx = robot.limelight.getTx();
                double ta = targetArea;

                String ballDirection;
                if (tx > CAPTURE_ALIGNMENT_DEG) {
                    ballDirection = "RIGHT";
                } else if (tx < -CAPTURE_ALIGNMENT_DEG) {
                    ballDirection = "LEFT";
                } else {
                    ballDirection = "AHEAD";
                }

                double steerPower = Range.clip(
                        BALL_STEER_SIGN * tx * RobotConstants.LL_AIM_KP,
                        -BALL_MAX_STEER_POWER,
                        BALL_MAX_STEER_POWER
                );

                double forwardPower;
                if (Math.abs(tx) <= CAPTURE_ALIGNMENT_DEG) {
                    forwardPower = BALL_APPROACH_POWER;
                } else if (Math.abs(tx) <= FORWARD_ALIGNMENT_DEG) {
                    forwardPower = BALL_PARTIAL_APPROACH_POWER;
                } else {
                    // Rotate toward the ball before driving forward.
                    forwardPower = 0.0;
                }

                robot.drive.arcadeDrive(forwardPower, steerPower);

                boolean intakeRunning =
                        ta >= START_INTAKE_AREA
                                && Math.abs(tx) <= FORWARD_ALIGNMENT_DEG;

                if (intakeRunning) {
                    robot.intake.intake();
                } else {
                    robot.intake.stop();
                }

                if (ta >= CAPTURE_AREA
                        && Math.abs(tx) <= CAPTURE_ALIGNMENT_DEG) {
                    captureStarted = true;
                    captureTimer.reset();
                }

                telemetry.addData("Phase", "Approaching yellow ball");
                telemetry.addData("Ball Direction", ballDirection);
                telemetry.addData("TX", "%.2f deg", tx);
                telemetry.addData("Target Area", "%.3f", ta);
                telemetry.addData(
                        "Intake / Capture TA",
                        "%.2f / %.2f",
                        START_INTAKE_AREA,
                        CAPTURE_AREA
                );
                telemetry.addData("Intake Running", intakeRunning);
                telemetry.addData(
                        "Drive / Steer",
                        "%.2f / %.2f",
                        forwardPower,
                        steerPower
                );
                telemetry.addData("Stable Target Frames", stableTargetFrames);
                addPipelineTelemetry();
                telemetry.update();
                idle();
                continue;
            }

            // No acceptable yellow target: continue scanning.
            robot.intake.stop();

            double requestedOffset = BALL_SEARCH_OFFSETS_DEG[searchWaypoint];
            double requestedHeading = searchCenterHeading + requestedOffset;
            double currentHeading = robot.odometry.getHeadingDeg();
            double headingError = wrapDegrees(requestedHeading - currentHeading);

            double requiredHoldTime = searchWaypoint == 0
                    ? INITIAL_AHEAD_HOLD_SEC
                    : SEARCH_WAYPOINT_HOLD_SEC;

            if (Math.abs(headingError) <= SEARCH_HEADING_TOLERANCE_DEG) {
                robot.drive.stop();

                if (waypointHoldTimer.seconds() >= requiredHoldTime) {
                    searchWaypoint++;

                    if (searchWaypoint >= BALL_SEARCH_OFFSETS_DEG.length) {
                        // Start another complete right/left scan.
                        searchWaypoint = 0;
                    }

                    waypointHoldTimer.reset();
                }
            } else {
                waypointHoldTimer.reset();

                double searchTurnMagnitude = Range.clip(
                        Math.abs(headingError) * SEARCH_HEADING_KP,
                        MIN_SEARCH_TURN_POWER,
                        MAX_SEARCH_TURN_POWER
                );

                double searchTurnPower =
                        ODOMETRY_TURN_SIGN
                                * Math.signum(headingError)
                                * searchTurnMagnitude;

                robot.drive.arcadeDrive(0.0, searchTurnPower);
            }

            telemetry.addData("Phase", "Searching for yellow ball");
            telemetry.addData("Search Offset", "%.1f deg", requestedOffset);
            telemetry.addData("Current Heading", "%.1f deg", currentHeading);
            telemetry.addData("Heading Error", "%.1f deg", headingError);
            telemetry.addData("Ball Visible", false);
            telemetry.addData("Target Area", "%.3f", targetArea);
            addPipelineTelemetry();
            telemetry.update();
            idle();
        }

        robot.drive.stop();
        robot.intake.stop();

        telemetry.addData(
                "Collection Result",
                pieceCollected
                        ? "Ball intake sequence completed"
                        : "Timed out without collecting ball"
        );
        telemetry.update();
    }

    private AprilTagDetection.TagInfo findTagById(
            List<AprilTagDetection.TagInfo> visibleTags,
            int targetId
    ) {
        for (AprilTagDetection.TagInfo tag : visibleTags) {
            if (tag.id == targetId) {
                return tag;
            }
        }
        return null;
    }

    private void addAprilTagTelemetry(
            List<AprilTagDetection.TagInfo> visibleTags,
            AprilTagDetection.TagInfo targetTag,
            double elapsedSeconds
    ) {
        telemetry.addData("LL Has Target", robot.limelight.hasTarget());
        telemetry.addData("Raw Target Area", "%.4f", robot.limelight.getTa());
        telemetry.addData("Tag " + TARGET_TAG_ID + " Visible", targetTag != null);
        telemetry.addData("Visible Tag Count", visibleTags.size());
        telemetry.addData("Approach Time", "%.1f sec", elapsedSeconds);

        StringBuilder visibleIds = new StringBuilder();
        for (AprilTagDetection.TagInfo tag : visibleTags) {
            if (visibleIds.length() > 0) {
                visibleIds.append(", ");
            }
            visibleIds.append(tag.id);
        }

        telemetry.addData(
                "Visible IDs",
                visibleIds.length() == 0 ? "none" : visibleIds.toString()
        );
    }

    private void addPipelineTelemetry() {
        if (robot.limelight.getLatest() != null) {
            telemetry.addData(
                    "Active Pipeline",
                    robot.limelight.getLatest().getPipelineIndex()
            );
        }
    }

    private double wrapDegrees(double degrees) {
        while (degrees > 180.0) {
            degrees -= 360.0;
        }
        while (degrees <= -180.0) {
            degrees += 360.0;
        }
        return degrees;
    }
}

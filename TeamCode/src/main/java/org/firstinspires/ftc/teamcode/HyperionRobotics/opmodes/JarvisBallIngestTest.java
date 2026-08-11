package org.firstinspires.ftc.teamcode.HyperionRobotics.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HyperionRobotics.HyperionRobot;
import org.firstinspires.ftc.teamcode.HyperionRobotics.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.HyperionRobotics.limelight.SensorLimelight3A;

/**
 * Hyperion autonomous:
 *
 * 1) Find AprilTag 12.
 * 2) Approach using Limelight + Pinpoint and stop ~24 inches away.
 * 3) Switch Pipeline 1 to Color / yellow-ball tracking.
 * 4) Sweep LEFT to -90 degrees, then RIGHT to +90 degrees.
 * 5) Track the selected yellow ColorTarget using TX.
 * 6) While visible, continuously estimate ball distance from TY.
 * 7) When the ball disappears near the intake, lock the last distance/heading.
 * 8) Use Pinpoint to drive the remaining calculated distance + 4 inches,
 *    intake at full power, with ZERO turn during final contact.
 *
 * NOTE:
 * The current TY range model uses the previously measured 18/24/36-inch
 * camera data. Replace/extend those calibration points after collecting
 * the new 2/4/6/8-ft color-pipeline measurements.
 */
@Autonomous(name = "Jarvis Ball Ingest Test", group = "Autonomous")
public class JarvisBallIngestTest extends LinearOpMode {

    private static final int TARGET_TAG_ID = 12;

    private HyperionRobot robot;
    private SensorLimelight3A limelight;

    // ---------------------------------------------------------------------
    // AprilTag approach
    // ---------------------------------------------------------------------

    private static final double TAG_STOP_DISTANCE_IN = 24.0;
    private static final double TAG_STOP_TOLERANCE_IN = 1.0;
    private static final int REQUIRED_STABLE_TAG_FRAMES = 3;
    private static final double MAX_TAG_APPROACH_TIME_SEC = 20.0;

    private static final double TAG_FAR_POWER = 0.45;
    private static final double TAG_MID_POWER = 0.25;
    private static final double TAG_NEAR_POWER = 0.13;

    private static final double TAG_STEER_KP = 0.020;
    private static final double TAG_MAX_STEER = 0.22;
    private static final double TAG_MIN_STEER = 0.12;

    private static final double TAG_HEADING_LOCK_TX_DEG = 2.0;
    private static final double TAG_HEADING_HOLD_KP = 0.012;
    private static final double TAG_HEADING_MAX_CORRECTION = 0.08;

    private static final double[] TAG_SEARCH_OFFSETS_DEG = {
            0.0, -15.0, -30.0, -45.0, -30.0, -15.0,
            0.0, 15.0, 30.0, 45.0, 30.0, 15.0
    };

    private static final double TAG_SEARCH_KP = 0.012;
    private static final double TAG_SEARCH_MIN_POWER = 0.14;
    private static final double TAG_SEARCH_MAX_POWER = 0.22;
    private static final double TAG_SEARCH_TOLERANCE_DEG = 4.0;
    private static final double TAG_SEARCH_HOLD_SEC = 0.20;
    private static final double MAX_TAG_DROPOUT_CONTINUE_SEC = 0.35;

    // ---------------------------------------------------------------------
    // Yellow-ball tracking
    // ---------------------------------------------------------------------

    private static final double MAX_BALL_COLLECTION_TIME_SEC = 30.0;
    private static final int REQUIRED_STABLE_BALL_FRAMES = 3;
    private static final int REQUIRED_CENTERED_FRAMES_FOR_COMMIT = 3;

    private static final double BALL_STRAIGHT_TX_DEG = 2.5;
    private static final double BALL_CENTERED_TX_DEG = 4.0;
    private static final double BALL_CREEP_TX_DEG = 12.0;
    private static final double BALL_INTAKE_START_TX_DEG = 12.0;

    private static final double BALL_FORWARD_POWER = 0.14;
    private static final double BALL_CREEP_POWER = 0.08;

    private static final double BALL_STEER_KP = 0.018;
    private static final double BALL_MIN_STEER = 0.12;
    private static final double BALL_MAX_STEER = 0.22;

    private static final double[] BALL_SEARCH_OFFSETS_DEG = {
            0.0, -30.0, -60.0, -90.0, -60.0, -30.0,
            0.0, 30.0, 60.0, 90.0, 60.0, 30.0
    };

    private static final double BALL_SEARCH_KP = 0.012;
    private static final double BALL_SEARCH_MIN_POWER = 0.14;
    private static final double BALL_SEARCH_MAX_POWER = 0.22;
    private static final double BALL_SEARCH_TOLERANCE_DEG = 5.0;
    private static final double BALL_SEARCH_INITIAL_HOLD_SEC = 0.50;
    private static final double BALL_SEARCH_HOLD_SEC = 0.25;

    private static final double ODOMETRY_TURN_SIGN = -1.0;

    // ---------------------------------------------------------------------
    // Vision -> Pinpoint ball handoff
    // ---------------------------------------------------------------------

    private static final double BALL_CLOSE_LOSS_MAX_DISTANCE_IN = 22.0;
    private static final double MAX_RECENT_BALL_LOSS_SEC = 0.30;

    private static final double BALL_CAPTURE_OVERTRAVEL_IN = 4.0;
    private static final double MAX_FINAL_PINPOINT_DRIVE_IN = 24.0;

    private static final double FINAL_CAPTURE_POWER = 0.10;
    private static final double FINAL_CAPTURE_TOLERANCE_IN = 0.35;
    private static final double MAX_FINAL_CAPTURE_TIME_SEC = 4.0;

    // ---------------------------------------------------------------------
    // Ball TY calibration
    // ---------------------------------------------------------------------

    private static final double BALL_TY_18_IN = -19.48;
    private static final double BALL_TY_24_IN = -16.87;
    private static final double BALL_TY_36_IN = -12.53;

    private static final double BALL_DISTANCE_18_IN = 18.0;
    private static final double BALL_DISTANCE_24_IN = 24.0;
    private static final double BALL_DISTANCE_36_IN = 36.0;

    private static final double BALL_DISTANCE_FILTER_ALPHA = 0.30;

    @Override
    public void runOpMode() {

        robot = new HyperionRobot(hardwareMap);
        limelight = new SensorLimelight3A(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("AprilTag", TARGET_TAG_ID);
        telemetry.addData("Tag stop", "%.1f in", TAG_STOP_DISTANCE_IN);
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) {
            limelight.stop();
            return;
        }

        boolean reachedTag = approachAprilTag();

        robot.drive.stop();

        if (!reachedTag || !opModeIsActive()) {
            robot.stopAll();
            limelight.stop();
            telemetry.addData("Status", "AprilTag approach failed / timed out");
            telemetry.update();
            sleep(1000);
            return;
        }

        telemetry.addData("Status", "Tag reached. Searching for yellow ball...");
        telemetry.update();

        boolean collected = collectYellowBall();

        robot.stopAll();
        limelight.stop();

        telemetry.addData(
                "Status",
                collected ? "Autonomous complete - ball captured"
                        : "Ball collection timed out"
        );
        telemetry.update();
        sleep(1000);
    }

    private boolean approachAprilTag() {

        limelight.useAprilTagPipeline();
        sleep(500);

        robot.updateSensors();
        limelight.update();

        final double searchCenterHeading = robot.odometry.getHeadingDeg();

        ElapsedTime approachTimer = new ElapsedTime();
        ElapsedTime searchHoldTimer = new ElapsedTime();
        ElapsedTime timeSinceTagSeen = new ElapsedTime();

        int searchWaypoint = 0;
        int stableFrames = 0;

        boolean hasSeenTag = false;
        boolean pinpointReferenceValid = false;
        boolean headingLockValid = false;

        double referenceX = Double.NaN;
        double referenceY = Double.NaN;
        double referenceDistance = Double.NaN;
        double lockedHeadingDeg = Double.NaN;

        while (opModeIsActive()
                && approachTimer.seconds() < MAX_TAG_APPROACH_TIME_SEC) {

            robot.updateSensors();
            limelight.update();

            double currentX = robot.odometry.getX();
            double currentY = robot.odometry.getY();
            double currentHeading = robot.odometry.getHeadingDeg();

            SensorLimelight3A.AprilTagTarget tag =
                    limelight.getAprilTag(TARGET_TAG_ID);

            if (tag == null) {

                stableFrames = 0;

                if (!hasSeenTag) {

                    double offset = TAG_SEARCH_OFFSETS_DEG[searchWaypoint];
                    double requestedHeading = searchCenterHeading + offset;
                    double headingError =
                            wrapDegrees(requestedHeading - currentHeading);

                    if (Math.abs(headingError)
                            <= TAG_SEARCH_TOLERANCE_DEG) {

                        robot.drive.stop();

                        if (searchHoldTimer.seconds() >= TAG_SEARCH_HOLD_SEC) {
                            searchWaypoint++;

                            if (searchWaypoint >= TAG_SEARCH_OFFSETS_DEG.length) {
                                searchWaypoint = 0;
                            }

                            searchHoldTimer.reset();
                        }

                    } else {

                        searchHoldTimer.reset();

                        double magnitude =
                                Range.clip(
                                        Math.abs(headingError) * TAG_SEARCH_KP,
                                        TAG_SEARCH_MIN_POWER,
                                        TAG_SEARCH_MAX_POWER
                                );

                        double turn =
                                ODOMETRY_TURN_SIGN
                                        * Math.signum(headingError)
                                        * magnitude;

                        robot.drive.arcadeDrive(0.0, turn);
                    }

                    telemetry.addData(
                            "Phase",
                            "Searching LEFT first for Tag %d",
                            TARGET_TAG_ID
                    );
                    telemetry.addData("Search Offset", "%.1f", offset);
                    telemetry.addData("Heading", "%.1f", currentHeading);
                    telemetry.update();
                    idle();
                    continue;
                }

                double lostTime = timeSinceTagSeen.seconds();

                if (pinpointReferenceValid
                        && headingLockValid
                        && lostTime <= MAX_TAG_DROPOUT_CONTINUE_SEC) {

                    double traveled =
                            Math.hypot(
                                    currentX - referenceX,
                                    currentY - referenceY
                            );

                    double pinpointRemaining =
                            referenceDistance
                                    - traveled
                                    - TAG_STOP_DISTANCE_IN;

                    if (pinpointRemaining <= TAG_STOP_TOLERANCE_IN) {
                        robot.drive.stop();
                        telemetry.addData(
                                "Phase",
                                "STOP - Pinpoint reached Tag offset"
                        );
                        telemetry.update();
                        return true;
                    }

                    double headingError =
                            wrapDegrees(lockedHeadingDeg - currentHeading);

                    double correction =
                            Range.clip(
                                    headingError * TAG_HEADING_HOLD_KP,
                                    -TAG_HEADING_MAX_CORRECTION,
                                    TAG_HEADING_MAX_CORRECTION
                            );

                    double drive =
                            Range.clip(
                                    pinpointRemaining * RobotConstants.LL_RANGE_KP,
                                    0.0,
                                    TAG_NEAR_POWER
                            );

                    robot.drive.arcadeDrive(drive, correction);

                    telemetry.addData(
                            "Phase",
                            "Tag dropout - Pinpoint continuation"
                    );
                    telemetry.addData(
                            "Pinpoint Remaining",
                            "%.2f in",
                            pinpointRemaining
                    );
                    telemetry.update();
                    idle();
                    continue;
                }

                robot.drive.stop();
                telemetry.addData("Phase", "Tag lost - waiting/reacquiring");
                telemetry.update();
                idle();
                continue;
            }

            hasSeenTag = true;
            timeSinceTagSeen.reset();
            searchHoldTimer.reset();

            double cameraDistance = tag.getDistanceInches();

            if (!Double.isFinite(cameraDistance)) {
                robot.drive.stop();
                telemetry.addData("Phase", "Tag visible but range invalid");
                telemetry.update();
                idle();
                continue;
            }

            stableFrames++;

            if (stableFrames < REQUIRED_STABLE_TAG_FRAMES) {
                robot.drive.stop();
                telemetry.addData(
                        "Phase",
                        "Validating Tag %d/%d",
                        stableFrames,
                        REQUIRED_STABLE_TAG_FRAMES
                );
                telemetry.addData("Camera Distance", "%.2f in", cameraDistance);
                telemetry.update();
                idle();
                continue;
            }

            double visionRemaining =
                    cameraDistance - TAG_STOP_DISTANCE_IN;

            if (visionRemaining <= TAG_STOP_TOLERANCE_IN) {
                robot.drive.stop();
                telemetry.addData(
                        "Phase",
                        "STOP - Vision reached 24-inch offset"
                );
                telemetry.addData("Camera Distance", "%.2f in", cameraDistance);
                telemetry.update();
                return true;
            }

            if (!pinpointReferenceValid
                    && Math.abs(tag.tx) <= TAG_HEADING_LOCK_TX_DEG) {

                referenceX = currentX;
                referenceY = currentY;
                referenceDistance = cameraDistance;
                lockedHeadingDeg = currentHeading;

                pinpointReferenceValid = true;
                headingLockValid = true;
            }

            if (Math.abs(tag.tx) <= TAG_HEADING_LOCK_TX_DEG) {
                lockedHeadingDeg = currentHeading;
                headingLockValid = true;
            }

            double pinpointRemaining = Double.NaN;

            if (pinpointReferenceValid) {

                double traveled =
                        Math.hypot(
                                currentX - referenceX,
                                currentY - referenceY
                        );

                pinpointRemaining =
                        referenceDistance
                                - traveled
                                - TAG_STOP_DISTANCE_IN;

                if (pinpointRemaining <= TAG_STOP_TOLERANCE_IN) {
                    robot.drive.stop();
                    telemetry.addData(
                            "Phase",
                            "STOP - Pinpoint reached 24-inch offset"
                    );
                    telemetry.addData("Camera Distance", "%.2f in", cameraDistance);
                    telemetry.addData(
                            "Pinpoint Remaining",
                            "%.2f in",
                            pinpointRemaining
                    );
                    telemetry.update();
                    return true;
                }
            }

            double controlRemaining = visionRemaining;

            if (Double.isFinite(pinpointRemaining)) {
                controlRemaining =
                        Math.min(
                                visionRemaining,
                                pinpointRemaining
                        );
            }

            controlRemaining = Math.max(0.0, controlRemaining);

            double visionSteer =
                    Range.clip(
                            tag.tx * TAG_STEER_KP,
                            -TAG_MAX_STEER,
                            TAG_MAX_STEER
                    );

            if (Math.abs(tag.tx)
                    > RobotConstants.LL_TX_TOLERANCE_DEG) {

                visionSteer =
                        applyMinimumMagnitude(
                                visionSteer,
                                TAG_MIN_STEER
                        );

            } else {
                visionSteer = 0.0;
            }

            double headingCorrection = 0.0;

            if (headingLockValid) {
                double headingError =
                        wrapDegrees(lockedHeadingDeg - currentHeading);

                headingCorrection =
                        Range.clip(
                                headingError * TAG_HEADING_HOLD_KP,
                                -TAG_HEADING_MAX_CORRECTION,
                                TAG_HEADING_MAX_CORRECTION
                        );
            }

            double steer =
                    Range.clip(
                            visionSteer + headingCorrection,
                            -TAG_MAX_STEER,
                            TAG_MAX_STEER
                    );

            double speedLimit;

            if (controlRemaining > 30.0) {
                speedLimit = TAG_FAR_POWER;
            } else if (controlRemaining > 10.0) {
                speedLimit = TAG_MID_POWER;
            } else {
                speedLimit = TAG_NEAR_POWER;
            }

            double drive =
                    Range.clip(
                            controlRemaining * RobotConstants.LL_RANGE_KP,
                            0.0,
                            speedLimit
                    );

            if (Math.abs(tag.tx) > 10.0) {
                drive = 0.0;
            } else if (Math.abs(tag.tx) > 5.0) {
                drive = Math.min(drive, TAG_NEAR_POWER);
            }

            robot.drive.arcadeDrive(drive, steer);

            telemetry.addData(
                    "Phase",
                    "Tracking Tag - Vision + Pinpoint"
            );
            telemetry.addData("Camera Distance", "%.2f in", cameraDistance);
            telemetry.addData("Vision Remaining", "%.2f in", visionRemaining);
            telemetry.addData(
                    "Pinpoint Remaining",
                    Double.isFinite(pinpointRemaining)
                            ? String.format("%.2f in", pinpointRemaining)
                            : "not initialized"
            );
            telemetry.addData("CONTROL Remaining", "%.2f in", controlRemaining);
            telemetry.addData("Tag TX", "%.2f deg", tag.tx);
            telemetry.addData("Drive / Steer", "%.2f / %.2f", drive, steer);
            telemetry.update();
            idle();
        }

        robot.drive.stop();
        return false;
    }

    private boolean collectYellowBall() {

        limelight.useGamePiecePipeline();
        sleep(500);

        robot.updateSensors();
        limelight.update();

        final double searchCenterHeading = robot.odometry.getHeadingDeg();

        int searchWaypoint = 0;
        int stableBallFrames = 0;
        int centeredFrames = 0;

        boolean hasTrackedBall = false;
        boolean finalCaptureStarted = false;

        double filteredBallDistance = Double.NaN;

        double lastBallTx = Double.NaN;
        double lastBallTy = Double.NaN;
        double lastBallArea = Double.NaN;
        double lastBallDistance = Double.NaN;
        double lastBallHeading = Double.NaN;

        double finalStartX = Double.NaN;
        double finalStartY = Double.NaN;
        double finalDriveDistance = Double.NaN;
        double finalLockedHeading = Double.NaN;

        ElapsedTime collectionTimer = new ElapsedTime();
        ElapsedTime waypointHoldTimer = new ElapsedTime();
        ElapsedTime timeSinceBallSeen = new ElapsedTime();
        ElapsedTime finalCaptureTimer = new ElapsedTime();

        robot.intake.stop();

        while (opModeIsActive()
                && collectionTimer.seconds() < MAX_BALL_COLLECTION_TIME_SEC) {

            robot.updateSensors();
            limelight.update();

            if (finalCaptureStarted) {

                robot.intake.intake();

                double currentX = robot.odometry.getX();
                double currentY = robot.odometry.getY();
                double currentHeading = robot.odometry.getHeadingDeg();

                double progress =
                        Math.hypot(
                                currentX - finalStartX,
                                currentY - finalStartY
                        );

                double remaining =
                        finalDriveDistance - progress;

                telemetry.addData(
                        "Phase",
                        "FINAL PINPOINT BALL CAPTURE"
                );
                telemetry.addData(
                        "Locked Ball Distance",
                        "%.2f in",
                        lastBallDistance
                );
                telemetry.addData(
                        "Overtravel",
                        "%.1f in",
                        BALL_CAPTURE_OVERTRAVEL_IN
                );
                telemetry.addData(
                        "Final Drive Target",
                        "%.2f in",
                        finalDriveDistance
                );
                telemetry.addData(
                        "Pinpoint Progress",
                        "%.2f in",
                        progress
                );
                telemetry.addData("Remaining", "%.2f in", remaining);
                telemetry.addData(
                        "Locked / Current Heading",
                        "%.1f / %.1f",
                        finalLockedHeading,
                        currentHeading
                );

                if (remaining <= FINAL_CAPTURE_TOLERANCE_IN) {
                    robot.drive.stop();
                    sleep(400);
                    telemetry.addData("Capture", "Completed");
                    telemetry.update();
                    return true;
                }

                if (finalCaptureTimer.seconds()
                        >= MAX_FINAL_CAPTURE_TIME_SEC) {

                    robot.drive.stop();
                    telemetry.addData("Capture", "Timed out");
                    telemetry.update();
                    return false;
                }

                /*
                 * Intentionally straight:
                 * no vision turn and no heading correction at final contact.
                 */
                robot.drive.arcadeDrive(
                        FINAL_CAPTURE_POWER,
                        0.0
                );

                telemetry.addData(
                        "Drive / Turn",
                        "%.2f / 0.00",
                        FINAL_CAPTURE_POWER
                );
                telemetry.update();
                idle();
                continue;
            }

            SensorLimelight3A.ColorTarget ball =
                    limelight.getBestColorTarget();

            boolean ballVisible =
                    ball != null
                            && limelight.isFresh(
                            RobotConstants.LL_MAX_RESULT_AGE_MS
                    );

            telemetry.addData("LL Raw Valid", limelight.isValid());
            telemetry.addData(
                    "Staleness",
                    "%d ms",
                    limelight.getStalenessMs()
            );
            telemetry.addData(
                    "Color Result Count",
                    limelight.getColorTargets().size()
            );
            telemetry.addData("Ball Visible", ballVisible);

            if (ballVisible) {

                hasTrackedBall = true;
                timeSinceBallSeen.reset();
                waypointHoldTimer.reset();

                stableBallFrames++;

                double rawDistance =
                        estimateBallDistanceFromTy(ball.ty);

                if (Double.isFinite(rawDistance)) {
                    if (!Double.isFinite(filteredBallDistance)) {
                        filteredBallDistance = rawDistance;
                    } else {
                        filteredBallDistance =
                                BALL_DISTANCE_FILTER_ALPHA * rawDistance
                                        + (1.0 - BALL_DISTANCE_FILTER_ALPHA)
                                        * filteredBallDistance;
                    }
                }

                lastBallTx = ball.tx;
                lastBallTy = ball.ty;
                lastBallArea = ball.area;

                if (Double.isFinite(filteredBallDistance)) {
                    lastBallDistance = filteredBallDistance;
                }

                lastBallHeading = robot.odometry.getHeadingDeg();

                if (Math.abs(ball.tx) <= BALL_CENTERED_TX_DEG) {
                    centeredFrames++;
                } else {
                    centeredFrames = 0;
                }

                if (Math.abs(ball.tx) <= BALL_INTAKE_START_TX_DEG) {
                    robot.intake.intake();
                } else {
                    robot.intake.stop();
                }

                if (stableBallFrames < REQUIRED_STABLE_BALL_FRAMES) {
                    robot.drive.stop();
                    telemetry.addData("Phase", "Confirming color ball");
                    addBallTelemetry(
                            ball,
                            filteredBallDistance,
                            stableBallFrames,
                            centeredFrames
                    );
                    telemetry.update();
                    idle();
                    continue;
                }

                double absTx = Math.abs(ball.tx);

                double drive;
                double steer;

                if (absTx <= BALL_STRAIGHT_TX_DEG) {

                    drive = BALL_FORWARD_POWER;
                    steer = 0.0;

                } else if (absTx <= BALL_CENTERED_TX_DEG) {

                    drive = BALL_FORWARD_POWER;

                    steer =
                            Range.clip(
                                    ball.tx * BALL_STEER_KP,
                                    -0.08,
                                    0.08
                            );

                } else if (absTx <= BALL_CREEP_TX_DEG) {

                    drive = BALL_CREEP_POWER;

                    steer =
                            Range.clip(
                                    ball.tx * BALL_STEER_KP,
                                    -BALL_MAX_STEER,
                                    BALL_MAX_STEER
                            );

                    steer =
                            applyMinimumMagnitude(
                                    steer,
                                    BALL_MIN_STEER
                            );

                } else {

                    drive = 0.0;

                    steer =
                            Range.clip(
                                    ball.tx * BALL_STEER_KP,
                                    -BALL_MAX_STEER,
                                    BALL_MAX_STEER
                            );

                    steer =
                            applyMinimumMagnitude(
                                    steer,
                                    BALL_MIN_STEER
                            );
                }

                robot.drive.arcadeDrive(drive, steer);

                telemetry.addData("Phase", "Tracking yellow ball");
                addBallTelemetry(
                        ball,
                        filteredBallDistance,
                        stableBallFrames,
                        centeredFrames
                );
                telemetry.addData(
                        "Drive / Steer",
                        "%.2f / %.2f",
                        drive,
                        steer
                );
                telemetry.addData(
                        "Intake Running",
                        Math.abs(ball.tx) <= BALL_INTAKE_START_TX_DEG
                );
                telemetry.update();
                idle();
                continue;
            }

            double lostFor =
                    timeSinceBallSeen.seconds();

            boolean recentlyLost =
                    hasTrackedBall
                            && lostFor <= MAX_RECENT_BALL_LOSS_SEC;

            boolean wasCentered =
                    centeredFrames
                            >= REQUIRED_CENTERED_FRAMES_FOR_COMMIT;

            boolean lastDistanceClose =
                    Double.isFinite(lastBallDistance)
                            && lastBallDistance
                            <= BALL_CLOSE_LOSS_MAX_DISTANCE_IN;

            if (recentlyLost
                    && wasCentered
                    && lastDistanceClose) {

                robot.drive.stop();
                robot.intake.intake();

                robot.updateSensors();

                finalStartX = robot.odometry.getX();
                finalStartY = robot.odometry.getY();
                finalLockedHeading = lastBallHeading;

                finalDriveDistance =
                        Range.clip(
                                lastBallDistance
                                        + BALL_CAPTURE_OVERTRAVEL_IN,
                                0.0,
                                MAX_FINAL_PINPOINT_DRIVE_IN
                        );

                finalCaptureStarted = true;
                finalCaptureTimer.reset();

                telemetry.addData(
                        "Phase",
                        "BALL LOST CLOSE -> PINPOINT COMMIT"
                );
                telemetry.addData(
                        "Last TX / TY / TA",
                        "%.2f / %.2f / %.4f",
                        lastBallTx,
                        lastBallTy,
                        lastBallArea
                );
                telemetry.addData(
                        "Calculated Distance",
                        "%.2f in",
                        lastBallDistance
                );
                telemetry.addData(
                        "Final Drive +4in",
                        "%.2f in",
                        finalDriveDistance
                );
                telemetry.update();
                idle();
                continue;
            }

            stableBallFrames = 0;
            centeredFrames = 0;
            robot.intake.stop();

            double requestedOffset =
                    BALL_SEARCH_OFFSETS_DEG[searchWaypoint];

            double requestedHeading =
                    searchCenterHeading + requestedOffset;

            double currentHeading =
                    robot.odometry.getHeadingDeg();

            double headingError =
                    wrapDegrees(
                            requestedHeading - currentHeading
                    );

            double holdTime =
                    searchWaypoint == 0
                            ? BALL_SEARCH_INITIAL_HOLD_SEC
                            : BALL_SEARCH_HOLD_SEC;

            if (Math.abs(headingError)
                    <= BALL_SEARCH_TOLERANCE_DEG) {

                robot.drive.stop();

                if (waypointHoldTimer.seconds() >= holdTime) {
                    searchWaypoint++;

                    if (searchWaypoint >= BALL_SEARCH_OFFSETS_DEG.length) {
                        searchWaypoint = 0;
                    }

                    waypointHoldTimer.reset();
                }

            } else {

                waypointHoldTimer.reset();

                double magnitude =
                        Range.clip(
                                Math.abs(headingError) * BALL_SEARCH_KP,
                                BALL_SEARCH_MIN_POWER,
                                BALL_SEARCH_MAX_POWER
                        );

                double turn =
                        ODOMETRY_TURN_SIGN
                                * Math.signum(headingError)
                                * magnitude;

                robot.drive.arcadeDrive(0.0, turn);
            }

            telemetry.addData(
                    "Phase",
                    "Searching LEFT/RIGHT for yellow ball"
            );
            telemetry.addData(
                    "Search Offset",
                    "%.1f deg",
                    requestedOffset
            );
            telemetry.addData(
                    "Current Heading",
                    "%.1f deg",
                    currentHeading
            );
            telemetry.addData(
                    "Heading Error",
                    "%.1f deg",
                    headingError
            );
            telemetry.addData(
                    "Last Tracked Distance",
                    Double.isFinite(lastBallDistance)
                            ? String.format("%.2f in", lastBallDistance)
                            : "none"
            );
            telemetry.update();
            idle();
        }

        robot.drive.stop();
        robot.intake.stop();
        return false;
    }

    /**
     * Current piecewise TY calibration.
     *
     * No TA threshold is used here.
     */
    private double estimateBallDistanceFromTy(double ty) {

        if (!Double.isFinite(ty)) {
            return Double.NaN;
        }

        if (ty <= BALL_TY_24_IN) {
            return linearInterpolate(
                    ty,
                    BALL_TY_18_IN,
                    BALL_DISTANCE_18_IN,
                    BALL_TY_24_IN,
                    BALL_DISTANCE_24_IN
            );
        }

        return linearInterpolate(
                ty,
                BALL_TY_24_IN,
                BALL_DISTANCE_24_IN,
                BALL_TY_36_IN,
                BALL_DISTANCE_36_IN
        );
    }

    private double linearInterpolate(
            double x,
            double x1,
            double y1,
            double x2,
            double y2
    ) {

        if (Math.abs(x2 - x1) < 1e-9) {
            return Double.NaN;
        }

        double fraction =
                (x - x1) / (x2 - x1);

        return y1 + fraction * (y2 - y1);
    }

    private void addBallTelemetry(
            SensorLimelight3A.ColorTarget ball,
            double distance,
            int stableFrames,
            int centeredFrames
    ) {

        telemetry.addData(
                "Color Result Count",
                limelight.getColorTargets().size()
        );

        telemetry.addData(
                "Ball TX / TY / TA",
                "%.2f / %.2f / %.4f",
                ball.tx,
                ball.ty,
                ball.area
        );

        telemetry.addData(
                "Calculated Ball Distance",
                Double.isFinite(distance)
                        ? String.format("%.2f in", distance)
                        : "n/a"
        );

        telemetry.addData(
                "Stable Frames",
                "%d / %d",
                stableFrames,
                REQUIRED_STABLE_BALL_FRAMES
        );

        telemetry.addData(
                "Centered Frames",
                "%d / %d",
                centeredFrames,
                REQUIRED_CENTERED_FRAMES_FOR_COMMIT
        );

        telemetry.addData(
                "Pipeline",
                "%d / %s",
                limelight.getActivePipelineIndex(),
                limelight.getActivePipelineType()
        );
    }

    private double applyMinimumMagnitude(
            double value,
            double minimumMagnitude
    ) {

        if (Math.abs(value) < 1e-9) {
            return 0.0;
        }

        if (Math.abs(value) < minimumMagnitude) {
            return Math.copySign(minimumMagnitude, value);
        }

        return value;
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
/* Copyright (c) 2026 Coach Chris Lemoine - Team00000 Mentor Bot
 *
 * Team00000 is a reference and demonstration platform, not a library for direct use.
 *
 * Purpose:
 *  - Coaches and mentors use this robot and codebase to test ideas, validate patterns,
 *    and demonstrate best preactices.
 *  - Students are expected to study this code and recreate equivalent functionality
 *    in their own team's codebase (team31192, team36103, team36104, etc.).
 *
 * We extract strong patters from the official samples and implenet them cleanly here
 * as a teaching reference rather than editing the external.samples directly.
 */

package org.firstinspires.ftc.team00000;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Hardware - Primary hardware abstraction for the mentor bot.
 *
 * <p>Provides a clean, reusable interface for controlling four mecanum drive motors
 * and reading pose data from the gobilda pinpoint odometry computer</p>
 *
 * <p>Students and OpModes should interact with this class through its public methods
 * rather than accessing motors or the Pinpoint directly</p>
 */
public class Hardware {

    /* =====================================================
     * HARDWARE OBJECTS
     * ===================================================== */
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private GoBildaPinpointDriver pinpoint = null;

    /* =====================================================
     * CONFIGURATION CONSTANTS
     *
     * These values are centralized here so they only need to be changed in one place.
     * Motor names must match the Driver Station configuration exactly.
     * ===================================================== */
    public static final String FRONT_LEFT_NAME = "frontLeftDrive";
    public static final String FRONT_RIGHT_NAME = "frontRightDrive";
    public static final String BACK_LEFT_NAME = "backLeftDrive";
    public static final String BACK_RIGHT_NAME = "backRightDrive";
    public static final String PINPOINT_NAME = "pinpoint";

    /*
     * PINPOINT PHYSICAL OFFSETS (in millimeters)
     *
     * These define the location of the Pinpoint relative to the robots center of rotation.
     * Positive X = forward from center. Positive Y = left from center (robot's perspective).
     *
     * These values MUST be measured on the physical robot after mounting.
     * Incorrect offsets are one of the most common causes of autonomous inaccuracy.
     */
    private static final double PINPOINT_X_OFFSET_MM = -96.00;
    private static final double PINPOINT_Y_OFFSET_MM = 92.00;
    private static final GoBildaOdometryPods PODS = GoBildaOdometryPods.goBILDA_4_BAR_POD;

    /*
     * ENCODER DIRECTIONS
     *
     * These frequently need to be tested and adjusted on the actual robot.
     * Incorrect directions will cause heading or strafe to be inverted.
     */
    private static final EncoderDirection X_ENCODER_DIRECTION = EncoderDirection.FORWARD;
    private static final EncoderDirection Y_ENCODER_DIRECTION = EncoderDirection.REVERSED;

    /* =====================================================
     * CONSTRUCTOR
     * ===================================================== */
    public Hardware(HardwareMap hardwareMap) {
        initDriveMotors(hardwareMap);
        initPinpoint(hardwareMap);
    }

    /* =====================================================
     * INITIALIZATION
     * ===================================================== */

    private void initDriveMotors (HardwareMap hardwareMap) {
        frontLeftDrive = hardwareMap.get(DcMotor.class, FRONT_LEFT_NAME);
        frontRightDrive = hardwareMap.get(DcMotor.class, FRONT_RIGHT_NAME);
        backLeftDrive = hardwareMap.get(DcMotor.class, BACK_LEFT_NAME);
        backRightDrive = hardwareMap.get(DcMotor.class, BACK_RIGHT_NAME);

        /*
         * MOTOR DIRECTIONS FOR STANDARD MECANUM (X-drive pattern)
         *
         * Viewed from above, the wheels should form an "X" roller pattern.
         * These directions assume direct drive (no extra gearing that reverses rotation).
         *
         * TEST PROCEDURE:
         *   Push the left stick forward
         *   - If the robot drives backward, flip the direction od ALL four motors.
         *   - If strafing is wrong, adjust the left vs right pairs.
         */
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Brake is more predictable than coast when using odometry for positioning.
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // We rely on the Pinpoint for Position, not the motor encoders.
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void initPinpoint (HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);

        pinpoint.setOffsets(PINPOINT_X_OFFSET_MM, PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(PODS);
        pinpoint.setEncoderDirections(X_ENCODER_DIRECTION, Y_ENCODER_DIRECTION);

        /*
         * Reset position and IMU at the start of every match / OpMode.
         *
         * In Iterative OpMode Style, any IMU setting time should be handled
         * explicitly in the init_loop() rather than with a blocking sleep here.
         * This keeps the hardware class lightweight during initialization.
         */
        pinpoint.resetPosAndIMU();
    }

    /* =====================================================
     * PUBLIC DRIVE API
     * ===================================================== */

    /**
     * Drives the robot in a **robot-centric** manner.
     * Movement is relative to the robot's current orientation.
     *
     * @param axial   Forward (+) / backward (-) power [-1.0, 1.0]
     * @param lateral Left (+) / right (-) strafe power [-1.0, 1.0]
     * @param yaw     Clockwise (+) / counter-clockwise (-) rotation power [-1.0, 1.0]
     */
    public void driveRobotCentric(double axial, double lateral, double yaw) {
        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial + lateral - yaw;

        /*
         * Normalize wheel powers so no individual wheel exceeds ±1.0.
         * This preserves the intended direction of travel even when
         * the combined request would otherwise saturate on or more motors.
         */
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        setDrivePower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    /**
     * Drives the robot in a **field-centric** manner.
     * "Forward" on the joystick always moves the robot toward the positive Y-axis on the field,
     * regardless of the robot's current rotation.
     *
     * @param axial   Forward (+) / backward (-) power relative to the field [-1.0, 1.0]
     * @param lateral Left (+) / right (-) strafe power relative to the field [-1.0, 1.0]
     * @param yaw     Clockwise (+) / counter-clockwise (-) rotation power [-1.0, 1.0]
     */
    public void driveFieldCentric(double axial, double lateral, double yaw) {
        double theta = Math.atan2(axial, lateral);
        double r = Math.hypot(lateral, axial);
        theta = AngleUnit.normalizeRadians(theta - getHeading(AngleUnit.RADIANS));

        // Rotate translational inputs by the inverse of the robot's current heading
        double rotatedAxial   = r * Math.sin(theta);
        double rotatedLateral = r * Math.cos(theta);

        driveRobotCentric(rotatedAxial, rotatedLateral, yaw);
    }

    /**
     * Directly sets power to each of the four drive motors.
     * Useful for debugging or when fine-grained control is needed.
     */
    public void setDrivePower(double frontLeft, double frontRight, double backLeft, double backRight) {
        frontLeftDrive.setPower(Range.clip(frontLeft, -1.0, 1.0));
        frontRightDrive.setPower(Range.clip(frontRight, -1.0, 1.0));
        backLeftDrive.setPower(Range.clip(backLeft, -1.0, 1.0));
        backRightDrive.setPower(Range.clip(backRight, -1.0, 1.0));
    }

    public void stopDrive() {
        setDrivePower(0.0, 0.0, 0.0, 0.0);
    }

    /* =====================================================
     * PUBLIC ODOMETRY / PINPOINT API
     * ===================================================== */

    /**
     * Updates the Pinpoint's internal pose calculations from the pods and IMU.
     * Must be called every loop iteration before reading pose data.
     */
    public void updatePose() {
        if (pinpoint != null) {
            pinpoint.update();
        }
    }

    public Pose2D getPose() {
        if (pinpoint != null) {
            return pinpoint.getPosition();
        }
        return new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);
    }

    public double getX(DistanceUnit unit) {
        return getPose().getX(unit);
    }

    public double getY(DistanceUnit unit) {
        return getPose().getY(unit);
    }

    public double getHeading(AngleUnit unit) {
        return getPose().getHeading(unit);
    }

    /**
     * Resets the Pinpoint position and IMU heading to zero.
     * Call this at the start of autonomous or whenever re-zeroing is desired.
     */
    public void resetPose() {
        if (pinpoint != null) {
            pinpoint.resetPosAndIMU();
        }
    }

    /**
     * Returns true if the Pinpoint has completed IMU calibration and is ready
     * to provide high-accuracy pose data.
     *
     * <p>In an Iterative OpMode, this can be polled inside init_loop() after
     * calling resetPose() when guaranteed IMU readiness is required before starting</p>
     */
    public boolean isPinpointReady() {
        if (pinpoint == null) return false;
        return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    /**
     * Triggers a fresh position + IMU recalibration on the Pinpoint.
     * Useful after disturbances or when re-zeroing is needed mid-match.
     */
    public void recalibratePinpoint() {
        if (pinpoint != null) {
            pinpoint.resetPosAndIMU();
        }
    }

    /**
     * Returns the raw GoBildaPinpointDriver for advanced use cases.
     * Most students should use the higher-level methods above instead.
     */
    public GoBildaPinpointDriver getPinpoint() {
        return pinpoint;
    }

    /* =====================================================
     * TELEMETRY HELPERS
     * ===================================================== */

    public void addDriveTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addData("Drive", "FL %.2f FR %.2f BL %.2f BR %.2f",
                frontLeftDrive.getPower(),
                frontRightDrive.getPower(),
                backLeftDrive.getPower(),
                backRightDrive.getPower());
    }

    public void addPoseTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry,
                                 DistanceUnit distUnit, AngleUnit angleUnit) {
        Pose2D pose = getPose();
        telemetry.addData("Pose", "X %.2f Y %.2f H %.1f",
                pose.getX(distUnit),
                pose.getY(distUnit),
                pose.getHeading(angleUnit));
    }
}
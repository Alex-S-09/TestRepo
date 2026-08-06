package org.firstinspires.ftc.teamcode.HyperionRobotics.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HyperionRobotics.constants.RobotConstants;

/**
 * goBILDA-style 6-wheel tank drivetrain (3 motors per side).
 * Supports arcade TeleOp and encoder-based Autonomous moves.
 */
public class DriveTrain {
    private final DcMotorEx leftFront;
    private final DcMotorEx leftMiddle;
    private final DcMotorEx leftBack;
    private final DcMotorEx rightFront;
    private final DcMotorEx rightMiddle;
    private final DcMotorEx rightBack;

    public DriveTrain(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, RobotConstants.LEFT_FRONT);
        leftMiddle = hardwareMap.get(DcMotorEx.class, RobotConstants.LEFT_MIDDLE);
        leftBack = hardwareMap.get(DcMotorEx.class, RobotConstants.LEFT_BACK);
        rightFront = hardwareMap.get(DcMotorEx.class, RobotConstants.RIGHT_FRONT);
        rightMiddle = hardwareMap.get(DcMotorEx.class, RobotConstants.RIGHT_MIDDLE);
        rightBack = hardwareMap.get(DcMotorEx.class, RobotConstants.RIGHT_BACK);

        // goBILDA convention: right side reversed for tank drive
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMiddle.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMiddle.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Arcade drive: forward (+y) and turn (+clockwise from driver view, stick x). */
    public void arcadeDrive(double drive, double turn) {
        double left = drive + turn;
        double right = drive - turn;
        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0) {
            left /= max;
            right /= max;
        }
        setTankPowers(left, right);
    }

    /** Tank drive with independent left/right stick values. */
    public void tankDrive(double leftPower, double rightPower) {
        setTankPowers(
                Range.clip(leftPower, -1.0, 1.0),
                Range.clip(rightPower, -1.0, 1.0));
    }

    public void setTankPowers(double left, double right) {
        leftFront.setPower(left);
        leftMiddle.setPower(left);
        leftBack.setPower(left);
        rightFront.setPower(right);
        rightMiddle.setPower(right);
        rightBack.setPower(right);
    }

    public void stop() {
        setTankPowers(0.0, 0.0);
    }

    /**
     * Drive a distance in inches using left-front / right-front encoders.
     * Blocking helper — call from Autonomous with opModeIsActive check outside.
     */
    public void driveInches(double inches, double power) {
        int target = (int) (inches * RobotConstants.COUNTS_PER_INCH);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setTargetPosition(target);
        rightFront.setTargetPosition(target);
        leftMiddle.setTargetPosition(target);
        rightMiddle.setTargetPosition(target);
        leftBack.setTargetPosition(target);
        rightBack.setTargetPosition(target);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setTankPowers(Math.abs(power), Math.abs(power));
    }

    public void turnDegreesApprox(double degrees, double power) {
        // Rough tank turn using wheel base estimate (~12 in). Tune for chassis.
        double inches = (degrees / 360.0) * (Math.PI * 12.0);
        int target = (int) (inches * RobotConstants.COUNTS_PER_INCH);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setTargetPosition(target);
        leftMiddle.setTargetPosition(target);
        leftBack.setTargetPosition(target);
        rightFront.setTargetPosition(-target);
        rightMiddle.setTargetPosition(-target);
        rightBack.setTargetPosition(-target);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setTankPowers(Math.abs(power), Math.abs(power));
    }

    public boolean isBusy() {
        return leftFront.isBusy() || rightFront.isBusy();
    }

    public void finishMotion() {
        stop();
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        leftMiddle.setMode(mode);
        leftBack.setMode(mode);
        rightFront.setMode(mode);
        rightMiddle.setMode(mode);
        rightBack.setMode(mode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFront.setZeroPowerBehavior(behavior);
        leftMiddle.setZeroPowerBehavior(behavior);
        leftBack.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
        rightMiddle.setZeroPowerBehavior(behavior);
        rightBack.setZeroPowerBehavior(behavior);
    }

    public int getLeftEncoder() {
        return leftFront.getCurrentPosition();
    }

    public int getRightEncoder() {
        return rightFront.getCurrentPosition();
    }
}

package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class BBMecanumDrive {
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
    private Telemetry telemetry;

    // IMU for navigation
    private IMU imu;

    //public Telemetry telemetry;

    public void init(HardwareMap hwMap) {
        // Declare motors
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        backRight = hwMap.get(DcMotor.class, "backRight");
        frontRight = hwMap.get(DcMotor.class, "frontRight");

        // Establishing the direction and mode for the motors
        // Direction
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);

        // Mode - set this to use encoders to prevent drift
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hwMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot hubOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

        imu.initialize(new IMU.Parameters(hubOrientation));
    }

    public void drive(double forward, double strafe, double rotate) {
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double backRightPower = forward + strafe - rotate;
        double frontRightPower = forward - strafe - rotate;

        // Normalizing power across motors to move evenly
        double maxPower = 1.0;
        double maxSpeed = 1.0; // Adjust for outreach or non-competition events

        // Get the max power from all motors and to set maxPower
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));

        // Scale power sent to motor by speed
        frontLeft.setPower(maxSpeed * (frontLeftPower / maxPower));
        backLeft.setPower(maxSpeed * (backLeftPower / maxPower));
        backRight.setPower(maxSpeed * (backRightPower / maxPower));
        frontRight.setPower(maxSpeed * (frontRightPower / maxPower));

        telemetry.addData("Left Motors",
            "FL: %.2f, BL: %.2f",
                    frontLeft.getPower(), backLeft.getPower());
        telemetry.addData("Right Motors",
                "BR: %.2f, FR: %.2f",
                backRight.getPower(), frontRight.getPower());
    }

    // Need to update to new fork to get the imu.getRobotYawPitchRollAngles method
    public void driveFieldRelative(double forward, double strafe, double rotate, Telemetry tele){
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);
        telemetry = tele;

        theta = AngleUnit.normalizeRadians(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newStrafe =  r * Math.cos(theta);

        telemetry.addLine().addData("Driving", "Fwd: %.2f, Stf: %.2f, Rot: %.2f",
                          newForward, newStrafe, rotate);
        this.drive(newForward, newStrafe, rotate);
    }
}
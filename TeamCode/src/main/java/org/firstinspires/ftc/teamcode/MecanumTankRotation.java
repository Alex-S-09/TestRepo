package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Mecanum Tank/Rotation Control OpMode
 * 
 * Logic:
 * - Only active when D-Pad Down is pressed.
 * - Restricts movement to 4 cardinal directions: Forward, Backward, Rotate CCW, Rotate CW.
 * - Left Stick Y: Forward/Backward
 * - Left Stick X (Left): Rotate Counter-Clockwise
 * - Right Stick X (Right): Rotate Clockwise
 */
@TeleOp(name = "Mecanum: Tank Rotation", group = "Control")
public class MecanumTankRotation extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    @Override
    public void runOpMode() {
        // Initialize Motors
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        // Set directions (Matching HardwareTest.java)
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized. Hold D-Pad Down for Tank/Rotation mode.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double lf = 0, rf = 0, lb = 0, rb = 0;

            // Only run if D-Pad Down is pressed
            if (gamepad1.dpad_down) {
                // Get inputs
                double drive = -gamepad1.left_stick_y;     // Positive is forward
                double rotCCW = -gamepad1.left_stick_x;   // Positive if stick is left
                double rotCW = gamepad1.right_stick_x;    // Positive if stick is right

                // Identify magnitudes for each cardinal direction
                double forwardVal = drive > 0 ? drive : 0;
                double backwardVal = drive < 0 ? -drive : 0;
                double ccwVal = rotCCW > 0 ? rotCCW : 0;
                double cwVal = rotCW > 0 ? rotCW : 0;

                // Find the dominant direction
                double max = Math.max(Math.max(forwardVal, backwardVal), Math.max(ccwVal, cwVal));

                if (max > 0.1) { // Deadzone check
                    if (max == forwardVal) {
                        // Forward
                        lf = rf = lb = rb = max;
                    } else if (max == backwardVal) {
                        // Backward
                        lf = rf = lb = rb = -max;
                    } else if (max == ccwVal) {
                        // Rotate Counter-Clockwise (Left side back, Right side forward)
                        lf = lb = -max;
                        rf = rb = max;
                    } else if (max == cwVal) {
                        // Rotate Clockwise (Left side forward, Right side back)
                        lf = lb = max;
                        rf = rb = -max;
                    }
                }
            }

            // Apply power to motors
            leftFront.setPower(lf);
            rightFront.setPower(rf);
            leftBack.setPower(lb);
            rightBack.setPower(rb);

            telemetry.addData("Mode", gamepad1.dpad_down ? "Tank Rotation" : "Idle (Hold D-Pad Down)");
            telemetry.addData("Powers", "LF:%.2f RF:%.2f LB:%.2f RB:%.2f", lf, rf, lb, rb);
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Mecanum Cardinal Control OpMode
 * 
 * Logic:
 * - Only active when D-Pad Up is pressed.
 * - Restricts movement to 4 cardinal directions: Forward, Backward, Strafe Left, Strafe Right.
 * - Left Stick Y: Forward/Backward
 * - Left Stick X (Left): Strafe Left
 * - Right Stick X (Right): Strafe Right
 */
@TeleOp(name = "Mecanum: Cardinal Control", group = "Control")
public class MecanumCardinalControl extends LinearOpMode {

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

        // Set directions
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized. Hold D-Pad Up for Mecanum mode.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double lf = 0, rf = 0, lb = 0, rb = 0;

            // Only run if D-Pad Up is pressed
            if (gamepad1.dpad_up) {
                // Get inputs
                double drive = -gamepad1.left_stick_y; // Positive is forward
                double strafeLeft = -gamepad1.left_stick_y; // Positive if stick is left
                double strafeRight = gamepad1.right_stick_y; // Positive if stick is right

                // We only want 4 directions. 
                // We'll find the dominant input among: Forward, Backward, Strafe Left, Strafe Right.
                
                double forwardVal = drive > 0 ? drive : 0;
                double backwardVal = drive < 0 ? -drive : 0;
                double leftVal = strafeLeft > 0 ? strafeLeft : 0;
                double rightVal = strafeRight > 0 ? strafeRight : 0;

                // Find the maximum of the 4 directions
                double max = Math.max(Math.max(forwardVal, backwardVal), Math.max(leftVal, rightVal));

                if (max > 0.1) { // Deadzone
                    if (max == forwardVal) {
                        // Forward
                        lf = rf = lb = rb = max;
                    } else if (max == backwardVal) {
                        // Backward
                        lf = rf = lb = rb = -max;
                    } else if (max == leftVal) {
                        // Strafe Left
                        // FL-, FR+, BL+, BR-
                        lf = -max;
                        rf = max;
                        lb = max;
                        rb = -max;
                    } else if (max == rightVal) {
                        // Strafe Right
                        // FL+, FR-, BL-, BR+
                        lf = max;
                        rf = -max;
                        lb = -max;
                        rb = max;
                    }
                }
            }

            // Apply power to motors
            leftFront.setPower(lf);
            rightFront.setPower(rf);
            leftBack.setPower(lb);
            rightBack.setPower(rb);

            telemetry.addData("Mode", gamepad1.dpad_up ? "Mecanum Cardinal" : "Idle");
            telemetry.addData("Powers", "LF:%.2f RF:%.2f LB:%.2f RB:%.2f", lf, rf, lb, rb);
            telemetry.update();
        }
    }
}

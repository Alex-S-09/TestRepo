package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Parallel Plate TeleOp", group = "TeleOp")
public class ParallelPlateTeleOp extends LinearOpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    @Override
    public void runOpMode() {
        leftMotor  = hardwareMap.get(DcMotor.class, "left_drive");
        rightMotor = hardwareMap.get(DcMotor.class, "right_drive");

        // One motor is mounted facing opposite direction, so reverse one side
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Ready — GameCube left stick controls forward/back");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // GameCube left stick Y: negative when pushed forward, so negate it
            double drive = -gamepad1.left_stick_y;

            leftMotor.setPower(drive);
            rightMotor.setPower(drive);

            telemetry.addData("Drive Power", "%.2f", drive);
            telemetry.addData("Direction", drive > 0.05 ? "Forward" : drive < -0.05 ? "Backward" : "Stopped");
            telemetry.update();
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}

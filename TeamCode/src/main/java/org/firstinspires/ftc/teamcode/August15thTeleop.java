package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class August15thTeleop extends LinearOpMode {
    DcMotor intake;

    @Override
    public void runOpMode() {
        intake = hardwareMap.dcMotor.get("intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0.1){
                intake.setPower(1);
            } else if (gamepad1.left_trigger > 0.1) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }


        }


    }
}
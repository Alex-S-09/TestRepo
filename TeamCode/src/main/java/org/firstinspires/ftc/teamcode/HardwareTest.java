package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Hardware Test OpMode
 * 
 * Hardware Configuration:
 * 1. GoBilda 5203 Motor: Configured as "motor"
 * 2. REV Color Sensor: Configured as "sensor_color"
 * 3. Servo: Configured as "test_servo"
 * 
 * Instructions:
 * - Use Left Stick Y to control the motor power.
 * - Use Gamepad buttons X and B to move the servo.
 * - View Color, Distance, and Servo data on the Driver Station telemetry.
 */
@TeleOp(name = "Hardware Test: Motor, Color & Servo", group = "Test")
public class HardwareTest extends LinearOpMode {

    private Servo servo;


    @Override
    public void runOpMode() {
        // Initialize Hardware
        // The strings "motor", "sensor_color", and "test_servo" must match your configuration
        servo = hardwareMap.get(Servo.class, "servo");


        servo.setPosition(0);


        telemetry.addData("Status", "Initialized. Press Play to start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Control Motor with Gamepad 1 Left Stick Y
            if(gamepad1.cross) {
                servo.setPosition(0);
            }
            else if(gamepad1.triangle) {
                servo.setPosition(0.5);
            }
            else if(gamepad1.circle) {
                servo.setPosition(1);
            }
        }
    }
}

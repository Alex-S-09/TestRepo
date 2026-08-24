package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

    private DcMotor motor;
    private NormalizedColorSensor colorSensor;
    private Servo servo;

    @Override
    public void runOpMode() {
        // Initialize Hardware
        // The strings "motor", "sensor_color", and "test_servo" must match your configuration
        motor = hardwareMap.get(DcMotor.class, "motor");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        servo = hardwareMap.get(Servo.class, "test_servo");

        // Optional: Set motor behavior
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Ensure the light is on for the color sensor
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        telemetry.addData("Status", "Initialized. Press Play to start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Control Motor with Gamepad 1 Left Stick Y
            double motorPower = -gamepad1.left_stick_y;
            motor.setPower(motorPower);

            // Control Servo with Gamepad 1 X and B
            if (gamepad1.x) {
                servo.setPosition(0.0);
            } else if (gamepad1.b) {
                servo.setPosition(1.0);
            } else if (gamepad1.y) {
                servo.setPosition(0.5);
            }

            // Get Color Data
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            
            // Display Motor Data
            telemetry.addData("Motor Power", "%.2f", motorPower);
            telemetry.addData("Motor Encoder", motor.getCurrentPosition());

            // Display Servo Data
            telemetry.addData("Servo Position", "%.2f", servo.getPosition());

            // Display Color Data
            telemetry.addLine("--- Color Sensor ---");
            telemetry.addData("Red", "%.3f", colors.red);
            telemetry.addData("Green", "%.3f", colors.green);
            telemetry.addData("Blue", "%.3f", colors.blue);
            telemetry.addData("Alpha", "%.3f", colors.alpha);

            // If the sensor supports distance, display it
            if (colorSensor instanceof DistanceSensor) {
                double distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
                telemetry.addData("Distance (cm)", "%.2f", distance);
            }

            telemetry.update();
        }
    }
}

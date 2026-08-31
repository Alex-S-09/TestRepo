package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Hardware Test OpMode - 4 Motor Drive
 * 
 * Hardware Configuration:
 * 1. 4 Motors: "left_front", "right_front", "left_back", "right_back"
 * 2. REV Color Sensor: Configured as "sensor_color"
 * 
 * Instructions:
 * - Use Left Stick Y to control the robot movement (Forward/Backward).
 * - View Color, Distance, and Motor data on the Driver Station telemetry.
 */
@TeleOp(name = "Hardware Test: 4 Motor Drive", group = "Test")
public class HardwareTest extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private NormalizedColorSensor colorSensor;

    @Override
    public void runOpMode() {
        // Initialize Drivetrain Motors
        // The names "left_front", "right_front", etc. must match your configuration
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        // Set motor directions (Left side usually needs to be reversed to move forward)
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // Optional: Set motor behavior to BRAKE for more controlled stopping
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Ensure the light is on for the color sensor
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        telemetry.addData("Status", "Initialized. Press Play to start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Control Motors with Gamepad 1 Left Stick Y
            // Forward is negative Y on most gamepads, so we negate it for positive power forward
            double drivePower = -gamepad1.left_stick_y;
            
            leftFront.setPower(drivePower);
            rightFront.setPower(drivePower);
            leftBack.setPower(drivePower);
            rightBack.setPower(drivePower);

            // Get Color Data
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            
            // Display Motor Data
            telemetry.addData("Drive Power", "%.2f", drivePower);
            telemetry.addData("LF Encoder", leftFront.getCurrentPosition());
            telemetry.addData("RF Encoder", rightFront.getCurrentPosition());

            // Display Color Data
            telemetry.addLine("--- Color Sensor ---");
            telemetry.addData("Red", "%.3f", colors.red);
            telemetry.addData("Green", "%.3f", colors.green);
            telemetry.addData("Blue", "%.3f", colors.blue);

            // If the sensor supports distance, display it
            if (colorSensor instanceof DistanceSensor) {
                double distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
                telemetry.addData("Distance (cm)", "%.2f", distance);
            }

            telemetry.update();
        }
    }
}

package org.firstinspires.ftc; // This tells the robot which folder this code belongs to

// These are like "Plugin Apps" that give the robot extra powers
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;              // The "High-tech Mouse" sensor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;               // To run without a controller
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;             // The basic way to write robot code
import com.qualcomm.robotcore.hardware.DcMotor;                          // To control the big wheels
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;    // For measuring turns (Degrees)
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit; // For measuring distance (Inches/MM)
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;       // To keep track of X, Y, and Angle

@Autonomous(name="Odometry_code") // This shows the code's name on your phone/driver station
public class Odometry_code extends LinearOpMode { // Our main robot "Brain"

    // These are the parts we will use
    // Our two motors for driving
    private DcMotor leftDrive, rightDrive;
    // Our "Pinpoint" sensor for tracking location
    private GoBildaPinpointDriver odo;

    // How fast we want the robot to go (0.0 to 1.0)
    static final double DRIVE_SPEED = 0.4;
    @Override
    public void runOpMode() { // This is where the magic starts!

        // We find our motors in the robot's hardware configuration
        leftDrive = hardwareMap.get(DcMotor.class, "back_left_motor");
        rightDrive = hardwareMap.get(DcMotor.class, "back_right_motor");

        // We set which way the wheels spin. One side is reversed so they work together!
        leftDrive.setDirection(DcMotor.Direction.REVERSE); 
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Tell the motors to just "run" without trying to count their own steps
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Find the Pinpoint sensor and tell it how our tracking wheels are set up
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        // Tell it we use 4-Bar pods
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // Spin direction
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // !!! UPDATE ME !!! Tell it where the sensor is located on the robot
        odo.setOffsets(0.0, 0.0, DistanceUnit.MM);
        // Reset the sensor so (0,0) is exactly where the robot is sitting right now
        odo.resetPosAndIMU();

        // Send a message to the controller/phone
        telemetry.addData("Status", "Ready");
        // Update the screen
        telemetry.update();

        // Wait for you to press the PLAY button
        waitForStart();

        // Only do this if the STOP button hasn't been pressed
        if (opModeIsActive()) {

            // ---- Forward path ----
            // Drive forward 10 inches
            driveInches(10, DRIVE_SPEED);
            // Take a short 0.3 second break
            sleep(300);
            // Turn to face 90 degrees (to the right)
            turnToAngle(90);
            sleep(300);
            // Drive forward 7 inches
            driveInches(7, DRIVE_SPEED);
            sleep(300);

            // ---- Reverse path ----
            // Drive backward 7 inches
            driveInches(-7, DRIVE_SPEED);
            sleep(300);
            // Turn back to face the original direction (0 degrees)
            turnToAngle(0);
            sleep(300);
            // Drive backward 10 inches
            driveInches(-10, DRIVE_SPEED);

            // Tell the team we finished!
            telemetry.addData("Status", "Done - back at start");
            // Ask the sensor for one last update
            odo.update();

            // Get our final position
            Pose2D finalPos = odo.getPosition();
            // Show final X, Y position and the final angle
            telemetry.addData("Final X", finalPos.getX(DistanceUnit.INCH));
            telemetry.addData("Final Y", finalPos.getY(DistanceUnit.INCH));
            telemetry.addData("Final Heading", finalPos.getHeading(AngleUnit.DEGREES));
            // Update the screen
            telemetry.update();
        }
    }

    // This helper makes the robot drive a certain distance
    public void driveInches(double distanceInches, double speed) {
        // "Proportional" gain - like a volume knob for power
        double kp = 0.05;
        // How close is "close enough" (0.5 inches)
        double tolerance = 0.5;
        // Check if we go forward (+) or backward (-)
        double direction = Math.signum(distanceInches);
        // Total distance to travel
        double targetDistance = Math.abs(distanceInches);

        // Refresh sensor data
        odo.update();
        // Save where we started
        Pose2D startPos = odo.getPosition();
        // Starting X and Y position
        double startX = startPos.getX(DistanceUnit.INCH);
        double startY = startPos.getY(DistanceUnit.INCH);

        // Keep driving until we reach the goal or stop
        while (opModeIsActive()) {
            // Constantly check our position
            odo.update();
            // Get current position
            Pose2D pos = odo.getPosition();
            double currentX = pos.getX(DistanceUnit.INCH);
            double currentY = pos.getY(DistanceUnit.INCH);

            // Calculate how far we have traveled from the start point
            double traveled = Math.hypot(currentX - startX, currentY - startY);
            // How much farther do we need to go?
            double error = targetDistance - traveled;

            // If we are close enough, stop!
            if (Math.abs(error) < tolerance) {
                setMotorPowers(0, 0); // Turn off motors
                // Exit the loop
                break;
            }

            // Calculate motor power based on the error (gets slower as we get closer)
            double power = error * kp * direction;
            // Don't go faster than our speed limit
            power = Math.max(-speed, Math.min(speed, power));
            // Give a little extra push if too slow
            if (Math.abs(power) < 0.15) power = Math.copySign(0.15, power);

            // Send power to the wheels
            setMotorPowers(power, power);

            telemetry.addData("Driving", "Target %.1f, Traveled %.1f", distanceInches, traveled);
            telemetry.update();
        }
    }

    // This helper makes the robot turn to a specific compass angle
    public void turnToAngle(double targetAngleDegrees) {
        // How sensitive the turning is
        double kp = 0.008;
        // How close to the angle is "close enough" (1 degree)
        double tolerance = 1.0;
        while (opModeIsActive()) {
            // Update sensor
            odo.update();
            // Get current position
            Pose2D pos = odo.getPosition();
            // Current angle
            double currentAngle = pos.getHeading(AngleUnit.DEGREES);

            // How far do we need to turn?
            double error = targetAngleDegrees - currentAngle;
            // Helper to find the shortest turn way
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            // Stop if we are pointing the right way
            if (Math.abs(error) < tolerance) {
                setMotorPowers(0, 0);
                break;
            }
            // Slow down as we get closer to the angle
            double power = error * kp;
            // Turning speed limit
            power = Math.max(-0.4, Math.min(0.4, power));
            // Minimum power to keep moving
            if (Math.abs(power) < 0.1) power = Math.copySign(0.1, power);

            // One wheel forward, one wheel back to SPIN!
            setMotorPowers(power, -power);

            telemetry.addData("Turning", "Target %.1f, Current %.1f", targetAngleDegrees, currentAngle);
            telemetry.update();
        }
    }

    // A simple way to set power to both motors at once
    private void setMotorPowers(double left, double right) {
        leftDrive.setPower(left);
        rightDrive.setPower(right);
    }
}

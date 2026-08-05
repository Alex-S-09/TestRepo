package org.firstinspires.ftc; // This tells the robot which folder this code belongs to

// These are like "Plugin Apps" that give the robot extra powers
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;              // The "High-tech Mouse" sensor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;               // To run without a controller
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;             // The basic way to write robot code
import com.qualcomm.robotcore.hardware.DcMotor;                          // To control the big wheels
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;    // For measuring turns (Degrees)
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit; // For measuring distance (Inches/MM)
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;       // To keep track of X, Y, and Angle

@Autonomous(name="Odometry_code") // This shows the code's name on your phone/driver station
public class Odometry_code extends LinearOpMode { // Our main robot "Brain"

    // These are the parts we will use
    // Our two motors =or driving
    private DcMotor back_left_motor, back_right_motor;
    // Our "Pinpoint" sensor for tracking location
    private GoBildaPinpointDriver odo;
    // Our Intake controller (from a separate file)
    private IntakeControl intake = new IntakeControl();

    // How fast we want the robot to go (0.0 to 1.0)
    static final double DRIVE_SPEED = 0.4;
    @Override
    public void runOpMode() { // This is where the magic starts!

        // We find our motors in the robot's hardware configuration
        back_left_motor = hardwareMap.get(DcMotor.class, "back_left_motor");
        back_right_motor = hardwareMap.get(DcMotor.class, "back_right_motor");

        // We set which way the wheels spin. One side is reversed so they work together!
        back_left_motor.setDirection(DcMotor.Direction.FORWARD);
        back_right_motor.setDirection(DcMotor.Direction.REVERSE);

        // Tell the motors to just "run" without trying to count their own steps
        back_left_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set motors to BRAKE so they stop immediately when power is 0
        back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Find the Pinpoint sensor and tell it how our tracking wheels are set up
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        // Tell it we use 4-Bar pods
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // Spin direction
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // !!! UPDATE ME !!! Tell it where the sensor is located on the robot
        odo.setOffsets(100.0, -40.0, DistanceUnit.MM);
        // Reset the sensor so (0,0) is exactly where the robot is sitting right now
        odo.resetPosAndIMU();

        // Initialize the intake controller
        intake.init(hardwareMap);

        // Send a message to the controller/phone
        telemetry.addData("Status", "Ready");
        telemetry.addData("Intake HW", intake.getStatus());
        telemetry.update();

        // Wait for you to press the PLAY button
        waitForStart();

        // Only do this if the STOP button hasn't been pressed
        if (opModeIsActive()) {

            // Start the intake and wait a moment for it to stabilize
            intake.start();
            sleep(500);

            // ---- Forward path ----
            // Drive forward 10 inches
            driveInches(48, DRIVE_SPEED);
            // Take a short 0.3 second break
            sleep(300);
            // Turn to face 90 degrees (to the right)
            turnToAngle(90);
            sleep(300);
            // Drive forward 7 inches
            driveInches(24, DRIVE_SPEED);
            sleep(300);

            // ---- Reverse path ----
            // Drive backward 7 inches
            driveInches(-24, DRIVE_SPEED);
            sleep(300);
            // Turn back to face the original direction (0 degrees)
            turnToAngle(0);
            sleep(300);
            // Drive backward 10 inches
            driveInches(48, DRIVE_SPEED);

            // Stop the intake when autonomous is done
            intake.stop();

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
        // Check if we go forward (+) or backward (-)f
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
            odo.update();
            Pose2D pos = odo.getPosition();
            double currentX = pos.getX(DistanceUnit.INCH);
            double currentY = pos.getY(DistanceUnit.INCH);

            double traveled = Math.hypot(currentX - startX, currentY - startY);
            double error = targetDistance - traveled;

            if (Math.abs(error) < tolerance) {
                setMotorPowers(0, 0);
                break;
            }

            double power = error * kp * direction;
            power = Math.max(-speed, Math.min(speed, power));
            if (Math.abs(power) < 0.15) power = Math.copySign(0.15, power);

            setMotorPowers(power, power);

            telemetry.addData("Driving", "Target %.1f, Traveled %.1f", distanceInches, traveled);
            telemetry.update();
        }
    }

    // This helper makes the robot turn to a specific compass angle
    public void turnToAngle(double targetAngleDegrees) {

        // START CONSERVATIVE - tune these later
        double kP = 0.006;
        double kD = 0.0008; // works on degrees/second, NOT error-per-loop

        double maxPower = 0.25;

        double positionTolerance = 1.5;   // degrees
        double velocityTolerance = 3.0;   // degrees/sec
        double settleTime = 0.15;         // seconds

        ElapsedTime totalTimer = new ElapsedTime();
        ElapsedTime settleTimer = new ElapsedTime();

        boolean settling = false;

        while (opModeIsActive() && totalTimer.seconds() < 3.0) {

            odo.update();

            Pose2D pos = odo.getPosition();
            
            // Get the angular velocity directly from the driver (since getVelocity() does not exist)
            double angularVelocity = odo.getHeadingVelocity(org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit.DEGREES);

            double currentAngle = pos.getHeading(AngleUnit.DEGREES);

            double error = targetAngleDegrees - currentAngle;

            // Normalize to -180 ... +180
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            /*
            * We are only FINISHED if:
            * 1. Heading is close enough to the target
            * 2. Robot has nearly stopped rotating
            */
            if (Math.abs(error) < positionTolerance &&
                Math.abs(angularVelocity) < velocityTolerance) {
                setMotorPowers(0, 0);
                if (!settling) {
                    settleTimer.reset();
                    settling = true;
                }
                if (settleTimer.seconds() >= settleTime) {
                    break;
                }
            } else {
                settling = false;

                /*
                 * P = turn toward target
                 * D = oppose rotational velocity
                 * Target velocity is zero, therefore:
                 * damping = -angularVelocity * kD
                 */
                double power = (error * kP) - (angularVelocity * kD);

                power = Math.max(-maxPower, Math.min(maxPower, power));
                setMotorPowers(power, -power);
            }
            telemetry.addData("Target","%.2f", targetAngleDegrees);
            telemetry.addData("Heading", "%.2f", currentAngle);
            telemetry.addData("Error", "%.2f", error);
            telemetry.addData("Angular velocity","%.2f deg/s", angularVelocity);
            telemetry.update();
            idle();
        }

        // Always leave motors stopped
        setMotorPowers(0, 0);
    }

    // A simple way to set power to both motors at once
    private void setMotorPowers(double left, double right) {
        back_left_motor.setPower(left);
        back_right_motor.setPower(right);
    }
}

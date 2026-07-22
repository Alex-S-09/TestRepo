package org.firstinspires.ftc.team31192.auto;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team31192.Hardware;
@Disabled
@Autonomous(name = "DriveByPinpoint (Iterative)", group = "Auto")
public class Pinpoint extends OpMode {

    private Hardware hardware;

    private int step = 0;

    private static final double DRIVE_SPEED = 0.5;
    private static final double TURN_SPEED  = 0.4;

    @Override
    public void init() {
        hardware = new Hardware(hardwareMap);
        // TODO 1: Reset the Pinpoint pose
    }

    @Override
    public void loop() {
        // TODO 2: Update the Pinpoint pose every loop
        // hardware.updatePose();

        switch (step) {
            case 0: // Drive forward
                // TODO 3: Call driveStraight() and advance step when done
                break;

            case 1: // Turn
                // TODO 4: Call turnToHeading() and advance step when done
                break;

            case 2: // Drive again
                // TODO 5: Drive forward again
                break;

            case 3:
                hardware.stopDrive();
                break;
        }

        hardware.addPoseTelemetry(telemetry, DistanceUnit.INCH, AngleUnit.DEGREES);
        telemetry.update();
    }

    // TODO 6: Implement driveStraight() method (similar to production version)
    private boolean driveStraight(double inches, double power, double targetHeadingDeg) {
        return false; // Replace with actual logic
    }

    // TODO 7: Implement turnToHeading() method
    private boolean turnToHeading(double targetHeadingDeg, double power) {
        return false; // Replace with actual logic
    }

    // =====================================================
    // ENGINEERING PORTFOLIO PROMPT
    // =====================================================
    /*
     * Engineering Portfolio Prompt – Iterative Pinpoint Autonomous
     *
     * After completing this scaffold, answer the following in your engineering notebook:
     *
     * 1. Why is using a step-based system (switch statement) helpful when writing
     *    autonomous programs in an Iterative OpMode?
     *
     * 2. What are the benefits of separating driving logic into helper methods
     *    (`driveStraight` and `turnToHeading`) instead of writing everything inside `loop()`?
     *
     * 3. How does the `turnToHeading` method handle heading error normalization?
     *    Why is this important?
     *
     * 4. What would happen if you did not call `updatePose()` every loop?
     *
     * 5. (Design Process & Iteration) If you were to improve this autonomous,
     *    what is one behavior you would add next, and why?
     */
}
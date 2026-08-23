package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.BBMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;

@TeleOp
public class DriverOrientatedOp extends OpMode {
    BBMecanumDrive drive = new BBMecanumDrive();
    Intake intake = new Intake();

    // Drivetrain variables
    double forward;
    double strafe;
    double rotate;

    // Intake variables
    boolean intakeForward = false;
    boolean intakeBackward = false;
    Intake.Direction intakeDirection = Intake.Direction.STOPPED;

    @Override
    public void init() {
        drive.init(hardwareMap, this.telemetry);
        intake.init(hardwareMap);
    }

    @Override
    public void loop() {
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x * -1; // Inverted x input because left and right were being reversed
        rotate = gamepad1.right_stick_x;

        intakeForward = gamepad1.y;
        intakeBackward = gamepad1.a;

        drive.drive(forward, strafe, rotate);

        // Will need debouncing for intake functionality:
        // If we press the Y button
        if(intakeForward){
            // And intake is not running forwards
            if (intakeDirection != Intake.Direction.FORWARD) {
                // Start intake because we are stopped or are running backwards
                intake.runIntakeForwards();
                intakeDirection = Intake.Direction.FORWARD;
            }
            else { // We are already running forwards. A second press should stop the intake
                intake.stopIntake();
                intakeDirection = Intake.Direction.STOPPED;
            }
        }
        // If we press the A button
        if(intakeBackward){
            // And intake is not running backwards
            if (intakeDirection != Intake.Direction.BACKWARD) {
                // Start running intake backward
                intake.runIntakeBackwards();
                intakeDirection = Intake.Direction.BACKWARD;
            }
            else { // We are already running backwards. A second press should stop the intake
                intake.stopIntake();
                intakeDirection = Intake.Direction.STOPPED;
            }
        }
        boolean updated = telemetry.update();
        if (!updated){
            telemetry.addLine().addData("Last telemetry update failed", "");
        }
    }
}

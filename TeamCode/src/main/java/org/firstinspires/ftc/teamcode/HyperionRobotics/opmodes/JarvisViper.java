package org.firstinspires.ftc.teamcode.HyperionRobotics.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HyperionRobotics.HyperionRobot;
import org.firstinspires.ftc.teamcode.HyperionRobotics.viper.ViperArm;

/**
 * TeleOp OpMode: Jarvis Viper
 * 1. Drivetrain: Right stick (FWD/BWD), Left stick (Left/Right turn).
 * 2. Intake: Right Bumper (Toggle ON/OFF).
 * 3. Viper Arm Manual: Left Bumper (UP), Left Trigger (DOWN).
 * 4. Viper Arm Presets: D-pad (Top=1, Right=2, Down=3, Left=4), Circle=0.
 */
@TeleOp(name="Jarvis Viper", group="TeleOp")
public class JarvisViper extends LinearOpMode {

    private HyperionRobot robot;
    private boolean intakeActive = false;
    private boolean lastRbState = false;

    @Override
    public void runOpMode() {
        robot = new HyperionRobot(hardwareMap);

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- 1. DRIVETRAIN (Arcade Drive) ---
            // Request: Right stick = FWD/BWD, Left stick = Left/Right
            double drive = -gamepad1.right_stick_y;
            double turn = gamepad1.left_stick_x;
            robot.drive.arcadeDrive(drive, turn);

            // --- 2. INTAKE (Toggle via Right Bumper) ---
            if (gamepad1.right_bumper && !lastRbState) {
                intakeActive = !intakeActive; // Toggle state
                if (intakeActive) {
                    robot.intake.intake();
                    robot.intake.open();
                } else {
                    robot.intake.stop();
                    robot.intake.close();
                }
            }
            lastRbState = gamepad1.right_bumper;

            // --- 3. VIPER ARM (Manual Control) ---
            if (gamepad1.left_bumper) {
                robot.viper.jog(1.0); // Extend up
            } else if (gamepad1.left_trigger > 0.1) {
                robot.viper.jog(-1.0); // Retract down
            }

            // --- 4. VIPER ARM (Presets via D-pad & Circle) ---
            if (gamepad1.dpad_up) {
                robot.viper.setStage(ViperArm.Stage.LOW); // Stage 1
            } else if (gamepad1.dpad_right) {
                robot.viper.setStage(ViperArm.Stage.MID); // Stage 2
            } else if (gamepad1.dpad_down) {
                robot.viper.setStage(ViperArm.Stage.HIGH); // Stage 3
            } else if (gamepad1.dpad_left) {
                robot.viper.setStage(ViperArm.Stage.MAX); // Stage 4
            } else if (gamepad1.circle || gamepad1.b) {
                robot.viper.setStage(ViperArm.Stage.STOWED); // Stage 0
            }

            // --- 5. TELEMETRY ---
            telemetry.addData("Intake", intakeActive ? "RUNNING" : "OFF");
            telemetry.addData("Viper Ticks", robot.viper.getCurrentTicks());
            telemetry.addData("Viper Stage", robot.viper.getCurrentStage());
            telemetry.update();
        }

        robot.stopAll();
    }
}

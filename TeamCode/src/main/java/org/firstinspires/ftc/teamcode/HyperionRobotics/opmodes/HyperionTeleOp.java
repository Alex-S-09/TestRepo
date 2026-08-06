package org.firstinspires.ftc.teamcode.HyperionRobotics.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HyperionRobotics.HyperionRobot;
import org.firstinspires.ftc.teamcode.HyperionRobotics.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.HyperionRobotics.limelight.DriverAssistance;

/**
 * HyperionRobotics TeleOp — drive, intake, viper, and Limelight assists.
 *
 * Gamepad 1:
 *  left stick Y  — drive
 *  right stick X — turn
 *  A             — auto-aim (AprilTag)
 *  B             — align scoring
 *  X             — align hang bar
 *  Y             — drive to preset
 *  DPAD left     — keep square to field
 *  BACK          — cancel assist
 *
 * Gamepad 2:
 *  A / B         — intake in / out
 *  X             — intake stop
 *  LB / RB       — claw open / close
 *  DPAD up/down  — viper stage up / down
 *  left stick Y  — viper jog
 */
@TeleOp(name = "Hyperion TeleOp", group = "Hyperion")
public class HyperionTeleOp extends LinearOpMode {
    private HyperionRobot robot;
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot = new HyperionRobot(hardwareMap);
        telemetry.addLine("Hyperion TeleOp ready");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            robot.updateSensors();

            boolean assistActive = robot.driverAssist.getMode() != DriverAssistance.Mode.IDLE
                    || robot.autoAim.isActive();

            // ---- Driver assists (gamepad1) ----
            if (gamepad1.back) {
                robot.driverAssist.cancel();
            } else if (gamepad1.a) {
                robot.autoAim.start();
            } else if (gamepad1.b) {
                robot.driverAssist.alignScoring(null);
            } else if (gamepad1.x) {
                robot.driverAssist.alignHang();
            } else if (gamepad1.y) {
                robot.driverAssist.driveToPreset();
            } else if (gamepad1.dpad_left) {
                robot.driverAssist.keepSquare();
            }

            if (assistActive || robot.driverAssist.getMode() != DriverAssistance.Mode.IDLE) {
                if (robot.autoAim.isActive()) {
                    robot.autoAim.update();
                } else {
                    robot.driverAssist.update();
                }
            } else {
                double drive = -gamepad1.left_stick_y * RobotConstants.TELEOP_DRIVE_SCALE;
                double turn = gamepad1.right_stick_x * RobotConstants.TELEOP_TURN_SCALE;
                robot.drive.arcadeDrive(drive, turn);
            }

            // ---- Intake (gamepad2) ----
            if (gamepad2.a) {
                robot.intake.intake();
            } else if (gamepad2.b) {
                robot.intake.outtake();
            } else if (gamepad2.x) {
                robot.intake.stop();
            }

            if (gamepad2.left_bumper) {
                robot.intake.open();
            } else if (gamepad2.right_bumper) {
                robot.intake.close();
            }

            // ---- Viper ----
            if (gamepad2.dpad_up) {
                robot.viper.nextStage();
                sleep(200);
            } else if (gamepad2.dpad_down) {
                robot.viper.previousStage();
                sleep(200);
            } else {
                robot.viper.jog(-gamepad2.left_stick_y);
            }

            telemetry.addData("Runtime", "%.1f", runtime.seconds());
            telemetry.addData("Pose", "x=%.1f y=%.1f h=%.1f",
                    robot.odometry.getX(), robot.odometry.getY(), robot.odometry.getHeadingDeg());
            telemetry.addData("Assist", robot.driverAssist.getMode());
            telemetry.addData("LL tx/ta", "%.1f / %.2f", robot.limelight.getTx(), robot.limelight.getTa());
            telemetry.addData("Viper", "%s (%d)", robot.viper.getCurrentStage(), robot.viper.getCurrentTicks());
            telemetry.update();
        }

        robot.stopAll();
    }
}

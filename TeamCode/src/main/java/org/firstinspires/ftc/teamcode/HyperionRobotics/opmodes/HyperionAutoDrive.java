package org.firstinspires.ftc.teamcode.HyperionRobotics.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HyperionRobotics.HyperionRobot;
import org.firstinspires.ftc.teamcode.HyperionRobotics.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.HyperionRobotics.viper.ViperArm;

/**
 * Sample autonomous: drive out, correct pose with Limelight, score sequence stub.
 * Replace waypoints / tag IDs for your alliance starting position.
 */
@Autonomous(name = "Hyperion Auto Drive", group = "Hyperion")
public class HyperionAutoDrive extends LinearOpMode {
    private HyperionRobot robot;

    @Override
    public void runOpMode() {
        robot = new HyperionRobot(hardwareMap);
        robot.odometry.setPose(0, 0, 0);

        telemetry.addLine("Hyperion Auto ready");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) {
            return;
        }

        // 1) Leave starting wall
        driveBlocking(24.0, RobotConstants.AUTO_DRIVE_POWER);

        // 2) Correct odometry drift using Limelight MegaTag if visible
        robot.localization.correctOdometry();
        robot.updateSensors();
        telemetry.addData("Corrected pose", "x=%.1f y=%.1f h=%.1f",
                robot.odometry.getX(), robot.odometry.getY(), robot.odometry.getHeadingDeg());
        telemetry.update();

        // 3) Optional: drive toward nearest AprilTag
        robot.driveToAprilTag.clearTarget();
        ElapsedGuard guard = new ElapsedGuard(4000);
        while (opModeIsActive() && !guard.expired() && !robot.driveToAprilTag.update()) {
            robot.updateSensors();
            telemetry.addData("Driving to tag", "tx=%.1f", robot.limelight.getTx());
            telemetry.update();
        }
        robot.drive.stop();

        // 4) Raise viper and score stub
        robot.viper.setStage(ViperArm.Stage.HIGH);
        while (opModeIsActive() && !robot.viper.isAtTarget()) {
            idle();
        }
        robot.intake.open();
        sleep(400);
        robot.intake.outtake();
        sleep(600);
        robot.intake.stop();
        robot.intake.close();
        robot.viper.setStage(ViperArm.Stage.STOWED);

        // 5) Park
        driveBlocking(12.0, RobotConstants.AUTO_DRIVE_POWER);

        robot.stopAll();
    }

    private void driveBlocking(double inches, double power) {
        robot.drive.driveInches(inches, power);
        while (opModeIsActive() && robot.drive.isBusy()) {
            robot.updateSensors();
            idle();
        }
        robot.drive.finishMotion();
    }

    /** Simple millisecond timeout without importing extra utilities in the loop. */
    private static class ElapsedGuard {
        private final long endMs;

        ElapsedGuard(long durationMs) {
            endMs = System.currentTimeMillis() + durationMs;
        }

        boolean expired() {
            return System.currentTimeMillis() >= endMs;
        }
    }
}

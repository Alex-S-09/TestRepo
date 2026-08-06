package org.firstinspires.ftc.teamcode.HyperionRobotics;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HyperionRobotics.drive.DriveTrain;
import org.firstinspires.ftc.teamcode.HyperionRobotics.intake.Intake;
import org.firstinspires.ftc.teamcode.HyperionRobotics.limelight.AprilTagDetection;
import org.firstinspires.ftc.teamcode.HyperionRobotics.limelight.AutoAim;
import org.firstinspires.ftc.teamcode.HyperionRobotics.limelight.DriveToAprilTag;
import org.firstinspires.ftc.teamcode.HyperionRobotics.limelight.DriverAssistance;
import org.firstinspires.ftc.teamcode.HyperionRobotics.limelight.GamePieceDetection;
import org.firstinspires.ftc.teamcode.HyperionRobotics.limelight.Limelight;
import org.firstinspires.ftc.teamcode.HyperionRobotics.limelight.RobotLocalization;
import org.firstinspires.ftc.teamcode.HyperionRobotics.odometry.PinpointOdometry;
import org.firstinspires.ftc.teamcode.HyperionRobotics.viper.ViperArm;

/**
 * Aggregates all HyperionRobotics subsystems for OpModes.
 */
public class HyperionRobot {
    public final DriveTrain drive;
    public final PinpointOdometry odometry;
    public final Intake intake;
    public final ViperArm viper;
    public final Limelight limelight;
    public final AprilTagDetection aprilTags;
    public final DriveToAprilTag driveToAprilTag;
    public final GamePieceDetection gamePieces;
    public final AutoAim autoAim;
    public final RobotLocalization localization;
    public final DriverAssistance driverAssist;

    public HyperionRobot(HardwareMap hardwareMap) {
        drive = new DriveTrain(hardwareMap);
        odometry = new PinpointOdometry(hardwareMap);
        intake = new Intake(hardwareMap);
        viper = new ViperArm(hardwareMap);
        limelight = new Limelight(hardwareMap);
        aprilTags = new AprilTagDetection(limelight);
        driveToAprilTag = new DriveToAprilTag(limelight, aprilTags, drive);
        gamePieces = new GamePieceDetection(limelight, drive, intake);
        autoAim = new AutoAim(limelight, drive);
        localization = new RobotLocalization(limelight, odometry);
        driverAssist = new DriverAssistance(
                limelight, drive, odometry, autoAim, driveToAprilTag, localization);
    }

    public void updateSensors() {
        odometry.update();
        limelight.update();
    }

    public void stopAll() {
        drive.stop();
        intake.stop();
        viper.stop();
        driverAssist.cancel();
        limelight.stop();
    }
}

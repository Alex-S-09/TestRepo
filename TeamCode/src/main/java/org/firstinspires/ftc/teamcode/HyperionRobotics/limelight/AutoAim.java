package org.firstinspires.ftc.teamcode.HyperionRobotics.limelight;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HyperionRobotics.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.HyperionRobotics.drive.DriveTrain;

/**
 * One-button auto-aim: rotate until Limelight tx is within tolerance, then stop.
 */
public class AutoAim {
    private final Limelight limelight;
    private final DriveTrain drive;
    private boolean active = false;
    private boolean locked = false;

    public AutoAim(Limelight limelight, DriveTrain drive) {
        this.limelight = limelight;
        this.drive = drive;
    }

    public void start() {
        active = true;
        locked = false;
        limelight.setPipeline(RobotConstants.LL_PIPELINE_APRILTAG);
    }

    public void cancel() {
        active = false;
        locked = false;
        drive.stop();
    }

    public boolean isActive() {
        return active;
    }

    public boolean isLocked() {
        return locked;
    }

    /**
     * Call each loop while aiming. Returns true when perfectly aligned.
     */
    public boolean update() {
        if (!active) {
            return false;
        }

        limelight.update();
        if (!limelight.hasTarget()) {
            drive.stop();
            locked = false;
            return false;
        }

        double tx = limelight.getTx();
        if (Math.abs(tx) <= RobotConstants.LL_TX_TOLERANCE_DEG) {
            drive.stop();
            locked = true;
            active = false;
            return true;
        }

        double turn = Range.clip(
                -tx * RobotConstants.LL_AIM_KP,
                -RobotConstants.LL_MAX_AIM_POWER,
                RobotConstants.LL_MAX_AIM_POWER);
        drive.arcadeDrive(0.0, turn);
        locked = false;
        return false;
    }
}

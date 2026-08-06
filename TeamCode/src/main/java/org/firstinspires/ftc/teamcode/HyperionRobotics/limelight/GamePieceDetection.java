package org.firstinspires.ftc.teamcode.HyperionRobotics.limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HyperionRobotics.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.HyperionRobotics.drive.DriveTrain;
import org.firstinspires.ftc.teamcode.HyperionRobotics.intake.Intake;

import java.util.List;

/**
 * Colored game-piece detection via a Limelight color/Neural pipeline.
 * Centers the robot and can auto-steer the intake toward the piece.
 */
public class GamePieceDetection {
    private final Limelight limelight;
    private final DriveTrain drive;
    private final Intake intake;

    public GamePieceDetection(Limelight limelight, DriveTrain drive, Intake intake) {
        this.limelight = limelight;
        this.drive = drive;
        this.intake = intake;
    }

    public void useGamePiecePipeline() {
        limelight.setPipeline(RobotConstants.LL_PIPELINE_GAMEPIECE);
    }

    public boolean isPiecePresent() {
        limelight.update();
        return limelight.hasTarget() && limelight.getTa() >= RobotConstants.LL_TA_MIN;
    }

    public double getPieceTx() {
        return limelight.getTx();
    }

    public double getPieceTa() {
        return limelight.getTa();
    }

    /**
     * Center on the largest detected color/detector target and optionally run intake.
     * @return true when roughly centered on a piece
     */
    public boolean centerOnPiece(boolean runIntake) {
        useGamePiecePipeline();
        limelight.update();

        if (!isPiecePresent()) {
            drive.stop();
            intake.stop();
            return false;
        }

        // Prefer detector results when available; fall back to tx/ta.
        LLResult result = limelight.getLatest();
        double tx = limelight.getTx();
        List<LLResultTypes.DetectorResult> detectors =
                result != null ? result.getDetectorResults() : null;
        if (detectors != null && !detectors.isEmpty()) {
            LLResultTypes.DetectorResult best = detectors.get(0);
            for (LLResultTypes.DetectorResult d : detectors) {
                if (d.getTargetArea() > best.getTargetArea()) {
                    best = d;
                }
            }
            tx = best.getTargetXDegrees();
        }

        double steer = Range.clip(
                -tx * RobotConstants.LL_AIM_KP,
                -RobotConstants.LL_MAX_AIM_POWER,
                RobotConstants.LL_MAX_AIM_POWER);

        double forward = (Math.abs(tx) < 8.0) ? 0.25 : 0.0;
        drive.arcadeDrive(forward, steer);

        if (runIntake && Math.abs(tx) < RobotConstants.LL_TX_TOLERANCE_DEG * 2) {
            intake.open();
            intake.intake();
        }

        return Math.abs(tx) < RobotConstants.LL_TX_TOLERANCE_DEG;
    }
}

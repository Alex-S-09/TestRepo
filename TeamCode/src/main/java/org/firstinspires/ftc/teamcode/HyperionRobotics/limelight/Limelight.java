package org.firstinspires.ftc.teamcode.HyperionRobotics.limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HyperionRobotics.constants.RobotConstants;

/**
 * Shared Limelight 3A hardware wrapper.
 *
 * update() should be called once per robot loop.  The latest result is cached
 * so every subsystem reads the same frame.
 */
public class Limelight {

    private final Limelight3A limelight;
    private LLResult latest;

    public Limelight(HardwareMap hardwareMap) {

        limelight =
                hardwareMap.get(
                        Limelight3A.class,
                        RobotConstants.LIMELIGHT
                );

        limelight.setPollRateHz(100);

        limelight.pipelineSwitch(
                RobotConstants.LL_PIPELINE_APRILTAG
        );

        limelight.start();
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    public void setPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
    }

    /**
     * Poll once per robot loop and cache the newest Limelight result.
     */
    public LLResult update() {

        latest =
                limelight.getLatestResult();

        return latest;
    }

    public LLResult getLatest() {
        return latest;
    }

    /**
     * "Raw valid" only means Limelight says the parent pipeline result is
     * valid.  It does NOT by itself prove that the target is the desired ball.
     */
    public boolean hasTarget() {

        return latest != null
                && latest.isValid();
    }

    /**
     * Age of the cached result in milliseconds.
     */
    public long getStalenessMs() {

        if (latest == null) {
            return Long.MAX_VALUE;
        }

        return latest.getStaleness();
    }

    /**
     * True only when the parent result is valid and fresh.
     */
    public boolean hasFreshTarget(long maxAgeMs) {

        return hasTarget()
                && getStalenessMs() <= maxAgeMs;
    }

    public double getTx() {

        return hasTarget()
                ? latest.getTx()
                : 0.0;
    }

    public double getTy() {

        return hasTarget()
                ? latest.getTy()
                : 0.0;
    }

    public double getTa() {

        return hasTarget()
                ? latest.getTa()
                : 0.0;
    }

    public Limelight3A getDevice() {
        return limelight;
    }
}
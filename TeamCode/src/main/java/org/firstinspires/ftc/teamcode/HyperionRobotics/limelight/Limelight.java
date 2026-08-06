package org.firstinspires.ftc.teamcode.HyperionRobotics.limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HyperionRobotics.constants.RobotConstants;

/**
 * Shared Limelight 3A hardware wrapper. Other Limelight modules depend on this.
 */
public class Limelight {
    private final Limelight3A limelight;
    private LLResult latest;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, RobotConstants.LIMELIGHT);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(RobotConstants.LL_PIPELINE_APRILTAG);
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

    /** Poll once per loop; caches the latest result. */
    public LLResult update() {
        latest = limelight.getLatestResult();
        return latest;
    }

    public LLResult getLatest() {
        return latest;
    }

    public boolean hasTarget() {
        return latest != null && latest.isValid();
    }

    public double getTx() {
        return hasTarget() ? latest.getTx() : 0.0;
    }

    public double getTy() {
        return hasTarget() ? latest.getTy() : 0.0;
    }

    public double getTa() {
        return hasTarget() ? latest.getTa() : 0.0;
    }

    public Limelight3A getDevice() {
        return limelight;
    }
}

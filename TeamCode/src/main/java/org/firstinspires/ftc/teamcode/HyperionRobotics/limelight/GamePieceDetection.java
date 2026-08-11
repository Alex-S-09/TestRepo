package org.firstinspires.ftc.teamcode.HyperionRobotics.limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HyperionRobotics.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.HyperionRobotics.drive.DriveTrain;
import org.firstinspires.ftc.teamcode.HyperionRobotics.intake.Intake;

/**
 * Generic yellow-ball detection / tracking helper.
 *
 * DESIGN:
 *
 * Pipeline 1 is assumed to be configured specifically for the yellow ball.
 *
 * Therefore:
 *
 *  - result.isValid() means pipeline 1 currently sees a valid target.
 *  - TX is used for left/right steering.
 *  - TY and TA are available for telemetry only.
 *  - TA is NOT used to reject a target.
 *  - No detector class name is required.
 *  - No DetectorResult or ColorResult parsing is required.
 *
 * This allows a ball to remain trackable at any distance where the active
 * Limelight pipeline can still see it, even when TA becomes very small.
 */
public class GamePieceDetection {

    /**
     * Generic target container used by JarvisLimelight and other OpModes.
     */
    public static class PieceTarget {

        public final double tx;
        public final double ty;
        public final double ta;

        public final long stalenessMs;

        /**
         * Kept for compatibility with existing telemetry.
         * This implementation always reports "pipeline".
         */
        public final String source;

        /**
         * Kept for compatibility with existing telemetry.
         * No class filtering is performed, so this is always "yellow-ball".
         */
        public final String className;

        public PieceTarget(
                double tx,
                double ty,
                double ta,
                long stalenessMs,
                String source,
                String className
        ) {
            this.tx = tx;
            this.ty = ty;
            this.ta = ta;
            this.stalenessMs = stalenessMs;
            this.source = source;
            this.className = className;
        }
    }

    private final Limelight limelight;
    private final DriveTrain drive;
    private final Intake intake;

    public GamePieceDetection(
            Limelight limelight,
            DriveTrain drive,
            Intake intake
    ) {
        this.limelight = limelight;
        this.drive = drive;
        this.intake = intake;
    }

    /**
     * Switch Limelight to the yellow-ball pipeline.
     */
    public void useGamePiecePipeline() {

        limelight.setPipeline(
                RobotConstants.LL_PIPELINE_GAMEPIECE
        );
    }

    /**
     * Return the latest valid/fresh yellow-ball target.
     *
     * IMPORTANT:
     *
     * There is deliberately NO minimum target-area requirement.
     *
     * Example:
     *
     *     TX = -24.74
     *     TY = -11.78
     *     TA = 0.057
     *
     * is still a valid target if pipeline 1 reports result.isValid() and the
     * frame is fresh enough.
     *
     * @param maxAgeMs maximum acceptable Limelight result age in milliseconds
     * @return PieceTarget when pipeline result is valid/fresh; null otherwise
     */
    public PieceTarget getBestTarget(long maxAgeMs) {

        LLResult result =
                limelight.getLatest();

        if (result == null) {
            return null;
        }

        /*
         * Limelight raw validity is the primary target-presence test.
         *
         * Because pipeline 1 is dedicated to the yellow ball, we do not apply
         * an additional detector-class or target-area gate here.
         */
        if (!result.isValid()) {
            return null;
        }

        long staleness =
                result.getStaleness();

        /*
         * Reject old frames.  We want steering decisions to use current data.
         *
         * maxAgeMs <= 0 disables the freshness limit if that is ever useful
         * during debugging.
         */
        if (maxAgeMs > 0
                && staleness > maxAgeMs) {

            return null;
        }

        double tx =
                result.getTx();

        double ty =
                result.getTy();

        double ta =
                result.getTa();

        /*
         * Protect the drive controller against malformed numeric data.
         *
         * TA is allowed to be zero/small.  It is NOT a visibility threshold.
         */
        if (!Double.isFinite(tx)
                || !Double.isFinite(ty)
                || !Double.isFinite(ta)) {

            return null;
        }

        return new PieceTarget(
                tx,
                ty,
                ta,
                staleness,
                "pipeline",
                "yellow-ball"
        );
    }

    /**
     * Convenience overload using a conservative default freshness window.
     */
    public PieceTarget getBestTarget() {

        return getBestTarget(200);
    }

    /**
     * Update Limelight and report whether the yellow-ball pipeline currently
     * has a valid target.
     */
    public boolean isPiecePresent() {

        limelight.update();

        return getBestTarget(200) != null;
    }

    /**
     * Return current ball TX.
     *
     * Returns 0 when no fresh target exists.
     */
    public double getPieceTx() {

        PieceTarget target =
                getBestTarget(200);

        return target != null
                ? target.tx
                : 0.0;
    }

    /**
     * Return current ball TY.
     *
     * Returns 0 when no fresh target exists.
     */
    public double getPieceTy() {

        PieceTarget target =
                getBestTarget(200);

        return target != null
                ? target.ty
                : 0.0;
    }

    /**
     * Return current ball TA.
     *
     * Returns 0 when no fresh target exists.
     *
     * TA is telemetry/context only; it is not used as a target-presence gate.
     */
    public double getPieceTa() {

        PieceTarget target =
                getBestTarget(200);

        return target != null
                ? target.ta
                : 0.0;
    }

    /**
     * Reusable simple centering helper.
     *
     * This is not the full autonomous collection state machine in
     * JarvisLimelight.  It is a basic helper for testing / TeleOp.
     *
     * Behavior:
     *
     *  - Large TX: turn in place.
     *  - Moderate TX: creep while correcting.
     *  - Small TX: drive straight.
     *  - Intake runs when target is reasonably aligned.
     */
    public boolean centerOnPiece(boolean runIntake) {

        useGamePiecePipeline();

        limelight.update();

        PieceTarget target =
                getBestTarget(200);

        if (target == null) {

            drive.stop();

            if (runIntake) {
                intake.stop();
            }

            return false;
        }

        double tx =
                target.tx;

        double absTx =
                Math.abs(tx);

        double forwardPower;
        double steerPower;

        /*
         * Very close to image center:
         * drive straight and suppress small steering twitches.
         */
        if (absTx <= 2.5) {

            forwardPower = 0.14;
            steerPower = 0.0;

            /*
             * Reasonably centered:
             * drive forward with a gentle correction.
             */
        } else if (absTx <= 6.0) {

            forwardPower = 0.12;

            steerPower =
                    Range.clip(
                            tx * 0.018,
                            -0.08,
                            0.08
                    );

            /*
             * Moderate error:
             * creep forward and turn.
             */
        } else if (absTx <= 12.0) {

            forwardPower = 0.07;

            steerPower =
                    Range.clip(
                            tx * 0.018,
                            -0.22,
                            0.22
                    );

            steerPower =
                    applyMinimumMagnitude(
                            steerPower,
                            0.12
                    );

            /*
             * Large horizontal error:
             * turn first; do not drive past the ball.
             */
        } else {

            forwardPower = 0.0;

            steerPower =
                    Range.clip(
                            tx * 0.018,
                            -0.22,
                            0.22
                    );

            steerPower =
                    applyMinimumMagnitude(
                            steerPower,
                            0.12
                    );
        }

        drive.arcadeDrive(
                forwardPower,
                steerPower
        );

        if (runIntake) {

            /*
             * Start intake well before contact once the ball is within a
             * reasonable steering angle.
             */
            if (absTx <= 12.0) {

                intake.open();
                intake.intake();

            } else {

                intake.stop();
            }
        }

        return absTx
                <= RobotConstants.LL_TX_TOLERANCE_DEG;
    }

    /**
     * Apply a minimum non-zero magnitude while preserving sign.
     *
     * Useful on the six-wheel tank chassis where very small turn powers may
     * not overcome scrub/static friction.
     */
    private double applyMinimumMagnitude(
            double value,
            double minimumMagnitude
    ) {

        if (Math.abs(value) < 1e-9) {
            return 0.0;
        }

        if (Math.abs(value) < minimumMagnitude) {

            return Math.copySign(
                    minimumMagnitude,
                    value
            );
        }

        return value;
    }
}
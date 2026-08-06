package org.firstinspires.ftc.teamcode.HyperionRobotics.odometry;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.HyperionRobotics.constants.RobotConstants;

/**
 * goBILDA Pinpoint Odometry Computer wrapper.
 * Call {@link #update()} once per loop before reading pose.
 */
public class PinpointOdometry {
    private final GoBildaPinpointDriver pinpoint;

    public PinpointOdometry(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, RobotConstants.PINPOINT);

        // Pod offsets from robot center (mm). X forward, Y left (Pinpoint convention).
        pinpoint.setOffsets(
                RobotConstants.PINPOINT_X_OFFSET_MM,
                RobotConstants.PINPOINT_Y_OFFSET_MM,
                DistanceUnit.MM);

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.resetPosAndIMU();
    }

    public void update() {
        pinpoint.update();
    }

    public void reset() {
        pinpoint.resetPosAndIMU();
    }

    public void setPose(double xInches, double yInches, double headingDeg) {
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH,
                xInches,
                yInches,
                AngleUnit.DEGREES,
                headingDeg));
    }

    public Pose2D getPose() {
        return pinpoint.getPosition();
    }

    public double getX() {
        return getPose().getX(DistanceUnit.INCH);
    }

    public double getY() {
        return getPose().getY(DistanceUnit.INCH);
    }

    public double getHeadingDeg() {
        return getPose().getHeading(AngleUnit.DEGREES);
    }

    public double getHeadingRad() {
        return getPose().getHeading(AngleUnit.RADIANS);
    }

    /** Correct Pinpoint pose from an external source (e.g. Limelight localization). */
    public void correctPose(double xInches, double yInches, double headingDeg) {
        setPose(xInches, yInches, headingDeg);
    }

    public GoBildaPinpointDriver getDriver() {
        return pinpoint;
    }
}

package org.firstinspires.ftc.teamcode.HyperionRobotics.constants;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * HyperionRobotics — hardware map names and tunable constants.
 * Match device names to your Robot Controller configuration.
 */
public final class RobotConstants {
    private RobotConstants() {}

    // ---- Drivetrain Configuration ----
    public enum DriveType {
        TANK_2_MOTOR,   // LeftBack, RightBack
        TANK_4_MOTOR,   // Front and Back pairs
        TANK_6_MOTOR,   // Front, Middle, and Back pairs
        MECANUM         // 4 motors with strafe capability
    }

    public static final DriveType ACTIVE_DRIVE_TYPE = DriveType.TANK_2_MOTOR; // Change this to switch setups

    // ---- Drive Motors ----
    public static final String LEFT_FRONT = "leftFront";
    public static final String LEFT_MIDDLE = "leftMiddle";
    public static final String LEFT_BACK = "leftBack";
    public static final String RIGHT_FRONT = "rightFront";
    public static final String RIGHT_MIDDLE = "rightMiddle";
    public static final String RIGHT_BACK = "rightBack";

    public static final double DRIVE_TICKS_PER_REV = 537.7; // goBILDA 312 RPM Yellow Jacket
    public static final double WHEEL_DIAMETER_IN = 96.0 / 25.4; // 96mm
    public static final double DRIVE_GEAR_RATIO = 1.0;
    public static final double COUNTS_PER_INCH =
            (DRIVE_TICKS_PER_REV * DRIVE_GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER_IN);

    public static final double TELEOP_DRIVE_SCALE = 1.0;
    public static final double TELEOP_TURN_SCALE = 0.75;
    public static final double AUTO_DRIVE_POWER = 0.5;
    public static final double AUTO_TURN_POWER = 0.4;

    // ---- Intake ----
    public static final String INTAKE_MOTOR = "intakeMotor";
    public static final String INTAKE_SERVO_LEFT = "intakeServoLeft";
    public static final String INTAKE_SERVO_RIGHT = "intakeServoRight";

    public static final double INTAKE_IN_POWER = 0.85;
    public static final double INTAKE_OUT_POWER = -0.70;
    public static final double INTAKE_SERVO_OPEN = 0.15;
    public static final double INTAKE_SERVO_CLOSED = 0.85;

    // ---- Viper (4-stage) ----
    public static final String VIPER_MOTOR = "viperMotor";
    // Optional second motor for dual-drive slides; leave unused if single motor.
    public static final String VIPER_MOTOR_2 = "viperMotor2";
    public static final boolean VIPER_DUAL_MOTOR = false;

    public static final int VIPER_STAGE_0 = 0;
    public static final int VIPER_STAGE_1 = 900;
    public static final int VIPER_STAGE_2 = 1800;
    public static final int VIPER_STAGE_3 = 2700;
    public static final int VIPER_STAGE_4 = 3600;
    public static final int VIPER_MAX_TICKS = 3800;
    public static final double VIPER_POWER = 0.90;
    public static final int VIPER_TOLERANCE = 25;

    // ---- Pinpoint ----
    public static final String PINPOINT = "pinpoint";
    // Offsets of Pinpoint from robot center (mm). Update after mounting.
    public static final double PINPOINT_X_OFFSET_MM = 100.0;
    public static final double PINPOINT_Y_OFFSET_MM = -40.0;

    // ---- Limelight 3A ----
    public static final String LIMELIGHT = "limelight";
    public static final int LL_PIPELINE_APRILTAG = 0;
    public static final int LL_PIPELINE_GAMEPIECE = 1;
    public static final int LL_PIPELINE_DRIVER = 2;

    public static final double LL_TX_TOLERANCE_DEG = 1.5;
    public static final double LL_TA_MIN = 0.15;
    public static final double LL_AIM_KP = 0.025;
    public static final double LL_RANGE_KP = 0.04;
    public static final double LL_DESIRED_TAG_DISTANCE_IN = 12.0;
    public static final double LL_MAX_AIM_POWER = 0.35;
    public static final double LL_MAX_DRIVE_POWER = 0.40;

    // Preset field poses (inches, field-centric). Tune for your season field.
    public static final double FIELD_SCORING_X = 48.0;
    public static final double FIELD_SCORING_Y = 0.0;
    public static final double FIELD_HANG_X = 0.0;
    public static final double FIELD_HANG_Y = 60.0;
    public static final double FIELD_PRESET_X = 36.0;
    public static final double FIELD_PRESET_Y = 36.0;
}

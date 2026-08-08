package org.firstinspires.ftc.teamcode.HyperionRobotics.intake;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HyperionRobotics.constants.RobotConstants;

/**
 * Intake motor plus left/right continuous rotation intake servos (rollers).
 */
public class Intake {
    private final DcMotorEx intakeMotor;
    private final CRServo servoLeft;
    private final CRServo servoRight;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.INTAKE_MOTOR);
        servoLeft = hardwareMap.get(CRServo.class, RobotConstants.INTAKE_SERVO_LEFT);
        servoRight = hardwareMap.get(CRServo.class, RobotConstants.INTAKE_SERVO_RIGHT);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Mirror right servo if linked mechanically opposite
        servoRight.setDirection(CRServo.Direction.REVERSE);
        stop();
    }

    /** Start both the motor and the servos to intake game pieces. */
    public void intake() {
        intakeMotor.setPower(RobotConstants.INTAKE_IN_POWER);
        servoLeft.setPower(1.0);
        servoRight.setPower(1.0);
    }

    /** Reverse both the motor and the servos to eject game pieces. */
    public void outtake() {
        intakeMotor.setPower(RobotConstants.INTAKE_OUT_POWER);
        servoLeft.setPower(-1.0);
        servoRight.setPower(-1.0);
    }

    /** Stop all intake hardware. */
    public void stop() {
        intakeMotor.setPower(0.0);
        servoLeft.setPower(0.0);
        servoRight.setPower(0.0);
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public void setServoPowers(double left, double right) {
        servoLeft.setPower(left);
        servoRight.setPower(right);
    }

    /** For CR servos, 'open' just starts the rollers to pull in. */
    public void open() {
        intake();
    }

    /** For CR servos, 'close' stops the rollers. */
    public void close() {
        stop();
    }

    public boolean isRunning() {
        return Math.abs(intakeMotor.getPower()) > 0.05;
    }
}

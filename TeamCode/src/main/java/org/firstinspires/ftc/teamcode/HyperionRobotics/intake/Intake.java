package org.firstinspires.ftc.teamcode.HyperionRobotics.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HyperionRobotics.constants.RobotConstants;

/**
 * Intake motor plus left/right intake servos (claw or flap pair).
 */
public class Intake {
    private final DcMotorEx intakeMotor;
    private final Servo servoLeft;
    private final Servo servoRight;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.INTAKE_MOTOR);
        servoLeft = hardwareMap.get(Servo.class, RobotConstants.INTAKE_SERVO_LEFT);
        servoRight = hardwareMap.get(Servo.class, RobotConstants.INTAKE_SERVO_RIGHT);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Mirror right servo if linked mechanically opposite
        servoRight.setDirection(Servo.Direction.REVERSE);
        close();
        stop();
    }

    public void intake() {
        intakeMotor.setPower(RobotConstants.INTAKE_IN_POWER);
    }

    public void outtake() {
        intakeMotor.setPower(RobotConstants.INTAKE_OUT_POWER);
    }

    public void stop() {
        intakeMotor.setPower(0.0);
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public void open() {
        servoLeft.setPosition(RobotConstants.INTAKE_SERVO_OPEN);
        servoRight.setPosition(RobotConstants.INTAKE_SERVO_OPEN);
    }

    public void close() {
        servoLeft.setPosition(RobotConstants.INTAKE_SERVO_CLOSED);
        servoRight.setPosition(RobotConstants.INTAKE_SERVO_CLOSED);
    }

    public void setServoPositions(double left, double right) {
        servoLeft.setPosition(left);
        servoRight.setPosition(right);
    }

    public boolean isOpen() {
        return Math.abs(servoLeft.getPosition() - RobotConstants.INTAKE_SERVO_OPEN) < 0.05;
    }
}

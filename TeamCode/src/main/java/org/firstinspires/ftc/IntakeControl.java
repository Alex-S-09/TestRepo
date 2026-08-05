package org.firstinspires.ftc;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This class handles the intake hardware and logic.
 */
public class IntakeControl {
    private DcMotor intake_motor = null;
    private CRServo left_intake_servo = null;
    private CRServo right_intake_servo = null;

    public void init(HardwareMap hardwareMap) {
        // --- 1. FIND INTAKE MOTOR ---
        try {
           intake_motor = hardwareMap.get(DcMotor.class, "intake_motor");
        } catch (Exception e) { /* ignore */ }

        if (intake_motor != null) {
           intake_motor.setDirection(DcMotor.Direction.FORWARD);
        }

        // --- 2. FIND SERVOS (Checking both CR and Standard types) ---
        try {
            left_intake_servo = hardwareMap.get(CRServo.class, "left_intake_servo");
        } catch (Exception e) { /* ignore */ }

        try {
            right_intake_servo = hardwareMap.get(CRServo.class, "right_intake_servo");
        } catch (Exception e) { /* ignore */ }

        // Set directions for CR Servos
        if (left_intake_servo != null) left_intake_servo.setDirection(CRServo.Direction.FORWARD);
        if (right_intake_servo != null) right_intake_servo.setDirection(CRServo.Direction.REVERSE);
    }

    public void start() {
        if (intake_motor != null) {
            intake_motor.setPower(0.8);
        }

        // Drive CR Servos
        if (left_intake_servo != null) left_intake_servo.setPower(1.0);
        if (right_intake_servo != null) right_intake_servo.setPower(1.0);
    }

    public void stop() {
        if (intake_motor != null) intake_motor.setPower(0.0);
        if (left_intake_servo != null) left_intake_servo.setPower(0.0);
        if (right_intake_servo != null) right_intake_servo.setPower(0.0);
    }

    public String getStatus() {
        String s = "";
        s += (intake_motor != null) ? "intake_motor:OK " : "intake_motor:MISSING " ;
        s += (left_intake_servo != null) ? "Left Servo:OK " : "Left Servo:MISSING ";
        s += (right_intake_servo != null) ? "Right Servo:OK " : "Right Servo:MISSING ";
        return s;
    }
}

package org.firstinspires.ftc.team36103.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team36103.Hardware;
@Disabled
@TeleOp(name = "RobotCentric", group = "TeleOp")
public class RobotCentric extends OpMode {

    private Hardware hardware;

    @Override
    public void init() {
        hardware = new Hardware(hardwareMap);
        // TODO: Reset pose if needed
    }

    @Override
    public void loop(){
        hardware.updatePose();

        double axial   = -gamepad1.left_stick_y;
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        // TODO: Call the correct drive method (robot centric)

        // Telemetry
        hardware.addDriveTelemetry(telemetry);
        hardware.addPoseTelemetry(telemetry, DistanceUnit.MM, AngleUnit.DEGREES);
        telemetry.addData("Mode", "Robot Centric");
        telemetry.update();
    }

    // =====================================================
    // ENGINEERING PORTFOLIO PROMPT
    // =====================================================
    /*
     * Engineering Portfolio Prompt – Robot Centric TeleOp
     *
     * After completing this scaffold, answer the following in your engineering notebook:
     *
     * 1. Why is it important to call updatePose() every loop iteration before reading
     *    position or heading data from the Pinpoint?
     *
     * 2. What is the purpose of separating the drive logic into its own hardware class
     *    instead of writing the mecanum math directly inside the TeleOp?
     *
     * 3. How does adding clear telemetry (drive powers + pose data) help with debugging
     *    and driver practice? Give one specific example.
     *
     * 4. (Design Process) If you were to improve this TeleOp in the future, what is one
     *    change you would make and why?
     *
     * 5. How does practicing in Robot Centric mode first help you better understand
     *    Field Centric driving later?
     */
}
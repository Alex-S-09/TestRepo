package org.firstinspires.ftc.team31192.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team31192.Hardware;
@Disabled
@TeleOp(name = "FieldCentric", group = "TeleOp")
public class FieldCentric extends OpMode {

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
        telemetry.addData("Mode", "Field Centric");
        telemetry.update();
    }

    // =====================================================
    // ENGINEERING PORTFOLIO PROMPT
    // =====================================================
    /*
     * Engineering Portfolio Prompt – Field Centric TeleOp
     *
     * After completing this scaffold, answer the following in your engineering notebook:
     *
     * 1. What is the main difference between Robot Centric and Field Centric driving
     *    from the driver’s perspective?
     *
     * 2. Why do we need to use the robot’s current heading when implementing field-centric
     *    control? What problem does this solve?
     *
     * 3. How does the field-centric implementation build on top of the robot-centric method
     *    in the hardware class?
     *
     * 4. Which driving mode do you think will be more useful during a match and why?
     *    (Consider game elements, driver preference, and reliability.)
     *
     * 5. (Iteration) After testing both modes, what is one improvement you would make
     *    to either TeleOp and why?
     */
}

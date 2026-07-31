package org.firstinspires.ftc;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name="Odometry_code")
public class Odometry_code extends LinearOpMode {

    private DcMotor back_left_motor, back_right_motor
    private GoBildaPinpointDriver odo;

    static final double DRIVE_SPEED = 0.4;

    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.setOffsets(0.0, 0.0, DistanceUnit.MM);

        odo.resetPosAndIMU();

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // ---- Forward path ----
            driveInches(10, DRIVE_SPEED);
            sleep(300);
            turnToAngle(90);
            sleep(300);
            driveInches(7, DRIVE_SPEED);
            sleep(300);

            // ---- Reverse path
            driveInches(-7, DRIVE_SPEED);
            sleep(300);
            turnToAngle(0);
            sleep(300);
            driveInches(-10, DRIVE_SPEED);

            telemetry.addData("Status", "Done - back at start");
            odo.update();
            Pose2D finalPos = odo.getPosition();
            telemetry.addData("Final X", finalPos.getX(DistanceUnit.INCH));
            telemetry.addData("Final Y", finalPos.getY(DistanceUnit.INCH));
            telemetry.addData("Final Heading", finalPos.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
    }


    public void driveInches(double distanceInches, double speed) {
        double kp = 0.05;
        double tolerance = 0.5;
        double direction = Math.signum(distanceInches);
        double targetDistance = Math.abs(distanceInches);

        odo.update();
        Pose2D startPos = odo.getPosition();
        double startX = startPos.getX(DistanceUnit.INCH);
        double startY = startPos.getY(DistanceUnit.INCH);

        while (opModeIsActive()) {
            odo.update();
            Pose2D pos = odo.getPosition();
            double currentX = pos.getX(DistanceUnit.INCH);
            double currentY = pos.getY(DistanceUnit.INCH);

            double traveled = Math.hypot(currentX - startX, currentY - startY);
            double error = targetDistance - traveled;

            if (Math.abs(error) < tolerance) {
                setMotorPowers(0, 0);
                break;
            }

            double power = error * kp * direction;
            power = Math.max(-speed, Math.min(speed, power));
            if (Math.abs(power) < 0.15) power = Math.copySign(0.15, power);

            setMotorPowers((int) power, (int) power);

            telemetry.addData("Driving", "Target %.1f, Traveled %.1f", distanceInches, traveled);
            telemetry.update();
        }
    }

    private void setMotorPowers(int i, int i1) {
    }

    public void turnToAngle(double targetAngleDegrees) {
        double kp = 0.008;
        double tolerance = 1.0;

        while (opModeIsActive()) {
            odo.update();
            Pose2D pos = odo.getPosition();
            double currentAngle = pos.getHeading(AngleUnit.DEGREES);

            double error = targetAngleDegrees - currentAngle;
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            if (Math.abs(error) < tolerance) {
                setMotorPowers(0, 0);
                break;
            }

            double power = error * kp;
            power = Math.max(-0.4, Math.min(0.4, power));
            if (Math.abs(power) < 0.1) power = Math.copySign(0.1, power);

            setMotorPowers((int) power, (int) -power);

            telemetry.addData("Turning", "Target %.1f, Current %.1f", targetAngleDegrees, currentAngle);

        }

    }
}
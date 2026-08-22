package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;


@Autonomous(name = "Pedro Base Auto", group = "Autonomous")
public class BaseAuto extends LinearOpMode {

    protected Follower follower;

    protected Timer pathTimer;
    protected ElapsedTime opModeTimer;

    protected enum PathState {
        START,
        PARK,
        END
    }

    protected PathState pathState;

    // Helper method to easily change states and reset the timer
    public void setPathState(PathState state) {
        pathState = state;
        pathTimer.resetTimer();
    }



    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);
        pathTimer = new Timer();
        opModeTimer = new ElapsedTime();

        waitForStart();
        opModeTimer.reset();

        setPathState(PathState.START);

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            follower.update();

            autonomousStateMachine();
        }
    }

    protected void autonomousStateMachine() {
        // Autos are to be placed here
    }
}

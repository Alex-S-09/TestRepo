package org.firstinspires.ftc.teamcode.testing;
//
//import android.util.Log;
//
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.localization.Localizer;
//import com.pedropathing.localization.PoseTracker;
//import com.pedropathing.math.Vector;
//import com.pedropathing.paths.Path;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;
//
//import java.util.List;
//
//import dev.anygeneric.blazeftc.DummyPlugOpMode;
//import dev.anygeneric.blazeftc.PositionData;
//
//@TeleOp(name = "Example Pedro High Speed Localization")
//public class ExamplePedroSpeedLocalization extends DummyPlugOpMode {
//    @Override
//    public void runOpModeInBlaze() {
//        Drawing.init();
//        try {
//            Telemetry tele = JoinedTelemetry(initializeBlazeFTC(telemetry), PanelsTelemetry.ftcTelemetry);
//            List<LynxModule> mods = hardwareMap.getAll(LynxModule.class);
//            for (LynxModule i : mods)
//                i.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//            engageMotorAcceleration();//this sends cmds directly to blaze, skipping Java serialization
//            //we create the follower. NOTE that this uses the pinpoint java driver to set all your settings and offsets
//            Follower follower = Constants.createFollower(hardwareMap);
//            //we have to take the pinpoint out of pedro. currently kind of a no-op though
//            GoBildaPinpointDriver ppd = ((PinpointLocalizer) follower.poseTracker.getLocalizer()).getPinpoint();
//            waitForStart();
//            //we replace pedro's localizer with ours
//            SingleDataLocalizer sdl = new SingleDataLocalizer();
//            follower.poseTracker = new PoseTracker(sdl);
//            ElapsedTime elt = new ElapsedTime();
//            //this closure should be called every time we get new data. we have to do some gymnastics because
//            //pedro doesn't like us doing it this way.
//            //TODO replace the pinpointBus (0 rn) with the port number the pinpoint is in
//            engagePinpointAcceleration(ppd, true, 3, (it) -> {
//                tele.addData("loop time (ms)", elt.milliseconds());
//                Log.d("TELEMETRY", "loop time (ms): " + elt.milliseconds());
//                elt.reset();
//
//                sdl.lastData = it;
//                long t0 = System.currentTimeMillis();
//                follower.update();
//                tele.addData("Pedro time", System.currentTimeMillis() - t0);
//                Log.d("TELEMETRY", "Pedro time: " + (System.currentTimeMillis() - t0));
//
//                tele.addData("x,y", follower.getPose().getX() + ", " + follower.getPose().getY());
//                Log.d("TELEMETRY", "x,y: " + follower.getPose().getX() + ", " + follower.getPose().getY());
//                tele.update();
//                Drawing.drawDebug(follower);
//                return null;
//            });
//            follower.followPath(new Path(new BezierLine(new Pose(0, 0), new Pose(10, 0))));
//            runBlazeFTC(0);
//
//            while (!isStopRequested()) {
//                //it doesn't matter what you do here
//                sleep(20);
//            }
//        } catch (Exception e) {
//            e.printStackTrace();
//        }
//    }
//    public static class SingleDataLocalizer implements Localizer {
//        public PositionData lastData = new PositionData();
//
//        @Override
//        public Pose getPose() {
//            return new Pose(lastData.getXPosition()/25.4, lastData.getYPosition()/25.4, lastData.getDirection());
//        }
//
//        @Override
//        public Pose getVelocity() {
//            return new Pose(lastData.getXVelocity()/25.4, lastData.getYVelocity()/25.4, lastData.getAngVelocity());
//        }
//
//        @Override
//        public Vector getVelocityVector() {
//            return getVelocity().getAsVector();
//        }
//
//        @Override
//        public void setStartPose(Pose setStart) {
//            throw new RuntimeException("not implemented");
//        }
//
//        @Override
//        public void setPose(Pose setPose) {
//            throw new RuntimeException("not implemented");
//        }
//
//        @Override
//        public void update() {
//            //this doesn't do anything. we receive new data asynchronously
//        }
//
//        @Override
//        public double getTotalHeading() {throw new RuntimeException("not implemented");}
//        @Override
//        public double getForwardMultiplier() {throw new RuntimeException("not implemented");}
//        @Override
//        public double getLateralMultiplier() {throw new RuntimeException("not implemented");}
//        @Override
//        public double getTurningMultiplier() {throw new RuntimeException("not implemented");}
//        @Override
//        public void resetIMU() {}
//        @Override
//        public double getIMUHeading() {throw new RuntimeException("not implemented");}
//        @Override
//        public boolean isNAN() {return false;}
//    }
//}

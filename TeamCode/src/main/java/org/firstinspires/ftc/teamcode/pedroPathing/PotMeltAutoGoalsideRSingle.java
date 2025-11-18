package org.firstinspires.ftc.teamcode.pedroPathing;

import android.os.SystemClock;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
public abstract class PotMeltAutoGoalsideRSingle extends OpMode {
    DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    DcMotor intake;
    CRServo outake1, outake2;
    DcMotor launcherL, launcherR;
    CRServo feederL, feederR;

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(76.1, 79.1, Math.toRadians(218));
    private final Pose control1 = new Pose(51.4, 53);
    private final Pose launchPose = new Pose(68.7, 70.6, Math.toRadians(226));
    private final Pose intakePose = new Pose(57.1, 53.1, Math.toRadians(0));
    private final Pose grabPose = new Pose(70.9, 53.1, Math.toRadians(0));
    private final Pose parkPose = new Pose(58.8, 81.2, Math.toRadians(226));

    private Path launchPath1;
    private PathChain  parkPath, intakePath1, grabPath1, launchPath2, scorePickup2, grabPickup3, scorePickup3;

    public void sleep(long time) {
        SystemClock.sleep(time);
    }

    public void launch(float spool, float launch_duration, double power) {
        long spool_long = (long) (spool*1000);
        long launch_duration_long = (long) (launch_duration)*1000;
        launcherL.setPower(power);
        launcherR.setPower(-power);
        SystemClock.sleep(spool_long);
        feederL.setPower(-1);
        outake1.setPower(1);
        outake2.setPower(-1);
        intake.setPower(1);
        SystemClock.sleep(launch_duration_long);
        launcherL.setPower(0);
        launcherR.setPower(0);
        feederL.setPower(0);
        outake1.setPower(0);
        outake2.setPower(0);
        intake.setPower(0);
    }

    public void suck() {
        launcherL.setPower(-0.05);
        launcherR.setPower(0.05);
        //feederL.setPower(-0.4);
        intake.setPower(1);
    }

    public void no_suck() {
        launcherL.setPower(0);
        launcherR.setPower(0);
        feederL.setPower(0);
        intake.setPower(0);
    }

    public void purge() {
        feederL.setPower(-1);
        outake1.setPower(1);
        outake2.setPower(-1);
        intake.setPower(-1);
    }

    public void stop_purge() {
        feederL.setPower(0);
        outake1.setPower(0);
        outake2.setPower(0);
        intake.setPower(0);
    }

    public void buildPaths() {
        launchPath1 = new Path(new BezierLine(startPose, launchPose));
        launchPath1.setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading());

        intakePath1 = follower.pathBuilder()
                .addPath(new BezierCurve(launchPose, control1, intakePose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), intakePose.getHeading())
                .build();

        grabPath1 = follower.pathBuilder()
                .addPath(new BezierLine(intakePose, grabPose))
                .setLinearHeadingInterpolation(intakePose.getHeading(), grabPose.getHeading())
                .build();

        launchPath2 = follower.pathBuilder()
                .addPath(new BezierLine(grabPose, launchPose))
                .setLinearHeadingInterpolation(grabPose.getHeading(), launchPose.getHeading())
                .build();

        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, parkPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), parkPose.getHeading())
                .build();

        /*scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(grabPose, launchPose))
                .setLinearHeadingInterpolation(grabPose.getHeading(), launchPose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, pickup3Pose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, launchPose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), launchPose.getHeading())
                .build();*/
    }

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        outake1 = hardwareMap.get(CRServo.class, "outake1");
        outake2 = hardwareMap.get(CRServo.class, "outake2");
        launcherL = hardwareMap.get(DcMotor.class, "launcherL");
        launcherR = hardwareMap.get(DcMotor.class, "launcherR");
        feederL = hardwareMap.get(CRServo.class, "feeder1");
        feederR = hardwareMap.get(CRServo.class, "feeder2");
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        switch (pathState) {
            case 0:
                launcherL.setPower(0);
                launcherR.setPower(0);
                feederL.setPower(0);
                outake1.setPower(0);
                outake2.setPower(0);
                intake.setPower(0);
                follower.followPath(launchPath1);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    launch(2, 5, 1);
                    //purge();
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(parkPath);
                    setPathState(-1);
                }
                break;
            /*case 3:
                if (!follower.isBusy()) {
                    stop_purge();
                    suck();
                    follower.followPath(grabPath1);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    sleep(1000);
                    no_suck();
                    follower.followPath(launchPath2);
                    setPathState(5);
                }

                break;
            case 5:
                if (!follower.isBusy()) {
                    launch(2, 5, 1);
                    follower.followPath(parkPath);
                    setPathState(-1);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    setPathState(-1); // End state
                }
                break;*/
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();

        // Debug info
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();

        autonomousPathUpdate();
    }

    public int getPathState() {
        return pathState;
    }

    /** Override this in subclasses to do custom actions when path state changes **/
    protected void autonomousPathUpdate() {
        // Default: do nothing
    }
}
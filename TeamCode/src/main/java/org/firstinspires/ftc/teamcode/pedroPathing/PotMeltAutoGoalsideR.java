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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class PotMeltAutoGoalsideR extends OpMode {
    DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    DcMotor intake;
    CRServo wheel;
    DcMotorEx launcher1, launcher2, turret;
    Servo hood;

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(73.7, 79.8, Math.toRadians(38));
    private final Pose control1 = new Pose(51.4, 53);
    private final Pose launchPose = new Pose(52.5, 46.1, Math.toRadians(49));
    private final Pose intakePose = new Pose(57.1, 57, Math.toRadians(0));
    private final Pose grabPose = new Pose(73, 57, Math.toRadians(0));
    private final Pose parkPose = new Pose(60.3, 74.8, Math.toRadians(47));

    private Path launchPath1;
    private PathChain  parkPath, intakePath1, grabPath1, launchPath2, scorePickup2, grabPickup3, scorePickup3;

    public void sleep(long time) {
        SystemClock.sleep(time);
    }

    public void launch(double power) {
        launcher1.setVelocity(-power);
        launcher2.setVelocity(-power);
        SystemClock.sleep(500);
        intake.setPower(0.8);
        wheel.setPower(1);
        SystemClock.sleep(2000);
        launcher1.setVelocity(0);
        launcher2.setVelocity(0);
        intake.setPower(0);
        wheel.setPower(0);
    }

    public void suck() {
        wheel.setPower(-0.8);
        intake.setPower(0.6);
    }

    public void no_suck() {
        intake.setPower(-0.1);
        sleep(900);
        intake.setPower(0);
        //wheel.setPower(0);
    }

    public void purge() {
        intake.setPower(-1);
        wheel.setPower(-1);
    }

    public void stop_purge() {
        intake.setPower(0);
        wheel.setPower(0);
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
        launcher1 = hardwareMap.get(DcMotorEx.class, "launcher1");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        wheel = hardwareMap.get(CRServo.class, "wheel");
        hood = hardwareMap.get(Servo.class, "hood");
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher1.setVelocityPIDFCoefficients(5,0.1,0,11);
        launcher2.setVelocityPIDFCoefficients(5,0.1,0,11);

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
                launcher1.setPower(0);
                launcher2.setPower(0);
                turret.setPower(0);
                intake.setPower(0);
                wheel.setPower(0);
                hood.setPosition(0.73);
                launcher1.setVelocity(-1250);
                launcher2.setVelocity(-1250);
                follower.followPath(launchPath1);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    launch(1270);
                    purge();
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(intakePath1);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    stop_purge();
                    suck();
                    follower.followPath(grabPath1);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    launcher1.setVelocity(-1250);
                    launcher2.setVelocity(-1250);
                    sleep(1000);
                    //no_suck();
                    wheel.setPower(-0.6);
                    //intake.setPower(-0.1);
                    sleep(500);
                    follower.followPath(launchPath2);
                    setPathState(5);
                }

                break;
            case 5:
                if (!follower.isBusy()) {
                    launch(1270);
                    purge();
                    follower.followPath(parkPath);
                    stop_purge();
                    setPathState(-1);
                }
                break;
            /*case 6:
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
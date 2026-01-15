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

public abstract class PotMeltAutoBasesideR extends OpMode {
    DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    DcMotor intake;
    DcMotor transfer_motor;
    CRServo servo_front;
    DcMotorEx launcher;
    Servo flipper;

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(53.8, 5.7, Math.toRadians(90));
    private final Pose control1 = new Pose(43.9, 46.2);
    private final Pose launchPose = new Pose(50.5, 14.9, Math.toRadians(66));
    private final Pose parkPose = new Pose(67.1, 11.6, Math.toRadians(0));
    private final Pose intakePose1 = new Pose(62.7, 26, Math.toRadians(0));
    private final Pose grabPose1 = new Pose(78.3, 26, Math.toRadians(0));

    private Path launchPath1;
    private PathChain parkPath, grabPath1, intakePath1, launchPath2;

    public void launch(float spool, double power) {
        long spool_long = (long) (spool*1000);
        launcher.setVelocity(power);
        SystemClock.sleep(spool_long);
        intake.setPower(1);
        transfer_motor.setPower(-1);
        servo_front.setPower(-1);
        SystemClock.sleep(3000);
        intake.setPower(-1);
        transfer_motor.setPower(1);
        servo_front.setPower(1);
        launcher.setVelocity(power+30);
        flipper.setPosition(1);
        SystemClock.sleep(2000);
        flipper.setPosition(0);
        launcher.setVelocity(0);
        intake.setPower(0);
        transfer_motor.setPower(0);
        servo_front.setPower(0);
    }

    public void suck() {
        intake.setPower(1);
    }

    public void no_suck() {
        intake.setPower(0);
    }

    public void purge() {
        intake.setPower(-1);
        transfer_motor.setPower(1);
        servo_front.setPower(1);
        flipper.setPosition(0);
    }

    public void stop_purge() {
        intake.setPower(0);
        transfer_motor.setPower(0);
        servo_front.setPower(0);
    }

    public void buildPaths() {
        launchPath1 = new Path(new BezierLine(startPose, launchPose));
        launchPath1.setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading());

        intakePath1 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, intakePose1))
                .setLinearHeadingInterpolation(launchPose.getHeading(), intakePose1.getHeading())
                .build();

        grabPath1 = follower.pathBuilder()
                .addPath(new BezierLine(intakePose1, grabPose1))
                .setLinearHeadingInterpolation(intakePose1.getHeading(), grabPose1.getHeading())
                .build();

        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, parkPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), parkPose.getHeading())
                .build();

        launchPath2 = follower.pathBuilder()
                .addPath(new BezierLine(grabPose1, launchPose))
                .setLinearHeadingInterpolation(grabPose1.getHeading(), launchPose.getHeading())
                .build();
    }

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        transfer_motor = hardwareMap.get(DcMotor.class, "transfer_motor");
        servo_front = hardwareMap.get(CRServo.class, "servo_front");
        flipper = hardwareMap.get(Servo.class, "flipper");
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
                launcher.setPower(0);
                intake.setPower(0);
                transfer_motor.setPower(0);
                servo_front.setPower(0);
                flipper.setPosition(0);
                intake.setPower(0);
                follower.followPath(launchPath1);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    launch(2, 1600);
                    purge();
                    follower.followPath(intakePath1);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    stop_purge();
                    suck();
                    follower.followPath(grabPath1);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    no_suck();
                    follower.followPath(launchPath2);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    launch(2, 1600);
                    follower.followPath(parkPath);
                    setPathState(-1);
                }

                break;
            /*case 5:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup3, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3, true);  \
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
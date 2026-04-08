package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.function.Supplier;


@Configurable
@TeleOp(name = "Pedro Test Teleop", group = "TeleOp")
public class TeleopPedroTest extends OpMode {

    DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    DcMotor intake;
    DcMotorEx launcher1, launcher2, turretMotor;
    Servo hood;

    CRServo wheel;



    /*public static double integralSumL = 0;
    public static double lastErrorL = 0;

    public static ElapsedTime timerL = new ElapsedTime();

    public static double Kp = 0.07;
    public static double Ki = 0.02;
    public static double Kd = 0.01;*/
    public static ElapsedTime timer = new ElapsedTime();

    private boolean relocalizeToggle = false;
    private boolean relocalizeButtonToggle = false;
    double timeSinceLocalized = 0;

    Limelight3A limelight;
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

        intake = hardwareMap.get(DcMotor.class, "intake");

        wheel = hardwareMap.get(CRServo.class, "wheel");
        launcher1 = hardwareMap.get(DcMotorEx.class, "launcher1");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");

        hood = hardwareMap.get(Servo.class, "hood");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0); // Switch to pipeline number 0

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    boolean opModeIsStarted = false;

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        opModeIsStarted =true;

        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }






        //telemetry.addData("Launcher Speed", launcher1.getVelocity());
        //telemetry.addData("Launcher R Speed", launcherR.getVelocity());
        //telemetry.addData("Servo Speed", feederL.getPower());
        //telemetry.addData("Servo Speed", feederR.getPower());
        telemetry.update();


        if (gamepad2.left_bumper) {
            intake.setPower(-1);
            wheel.setPower(1);

            // adjust numbers to work where they put the intake on the robot
        }else if (gamepad2.right_bumper){
            intake.setPower(1);
            wheel.setPower(-1);

        }else{
            intake.setPower(0);
            wheel.setPower(0);
        }
        if (gamepad2.dpad_up) {
            hood.setPosition(0.58);
        }
        else if (gamepad2.dpad_down) {
            hood.setPosition(0.73);
        }

        LLResult result = limelight.getLatestResult();


        if(gamepad1.a){
            if (result != null) {
                if (result.isValid()) {
                    follower.setPose(botCameraPose);
                }
            }
        }

        relocalizationUpdate(limelight, telemetry);
        turretMovement(true);

        if (opModeIsStarted) {
            turretMotor.setTargetPosition((int) turretTargetGoal);
            telemetry.addData("turret Clicks: ", turretMotor.getCurrentPosition());

        }


        telemetry.addData("PedroFollower: ", follower.getPose());
        //telemetry.addData("Bot Camera Pose: ", botCameraPose.getPose());


        telemetry.update();



        launcher1.setVelocity(gamepad2.left_trigger*-1550);
        launcher2.setVelocity(gamepad2.left_trigger*-1550);

    }

    private Pose getFTCPoseAsPedro(Pose2D ftcPose) {
        return new Pose(ftcPose.getY(DistanceUnit.INCH) + 72, Math.abs(ftcPose.getX(DistanceUnit.INCH) - 72), follower.getHeading());
    }

    Pose botCameraPose;
    public void relocalizationUpdate(Limelight3A limelight, Telemetry telemetry){
        LLResult result = limelight.getLatestResult();
        Pose2D botpose2D;
        Pose botPoseAsPedro;

        if (result != null) {
            if (result.isValid()) {

                relocalizeToggle = true;

                botpose2D = new Pose2D(DistanceUnit.INCH, result.getBotpose().getPosition().x * 39.37008, result.getBotpose().getPosition().y * 39.37008, AngleUnit.RADIANS, PedroComponent.follower().getHeading());
                botPoseAsPedro = getFTCPoseAsPedro(botpose2D);

                telemetry.addData("Limelight Coordinates As Pedro: ", getFTCPoseAsPedro(botpose2D));

                botCameraPose = new Pose(botPoseAsPedro.getX(), botPoseAsPedro.getY(), PedroComponent.follower().getHeading());
            } else {
                relocalizeToggle = false;
            }

        }
    }

    private double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle > 180) return angle - 360;
        if (angle < -180) return angle + 360;
        return angle;
    }

    public double degreesToTurnCorrected = 0;
    public Pose targetPoseBlue = new Pose(5, 142);
    public Pose targetPoseRed = new Pose(140, 140);
    public double encoderClicksPerDeg = 360d / 1445.58d; //limits: -1197, 1197
    public double degsPerClick = 1445.58d / 360d;


    double turretTargetGoal = 0;
    public void turretMovement(boolean isRed){
        // Select target
        Pose targetPosition = isRed ? targetPoseRed : targetPoseBlue;

        // Robot pose
        Pose robotPose = follower.getPose();

        // Absolute angle from robot to target in degrees
        double targetAngleDeg = Math.toDegrees(Math.atan2(
                targetPosition.getY() - robotPose.getY(),
                targetPosition.getX() - robotPose.getX()
        ));

        // Convert robot heading to degrees
        double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());

        // Turret angle relative to field
        double turretCurrentDeg = turretMotor.getCurrentPosition() * encoderClicksPerDeg + robotHeadingDeg;

        // Compute path difference
        double deltaDeg = normalizeAngle(targetAngleDeg - turretCurrentDeg);

        // Set absolute target in encoder ticks
        turretTargetGoal = turretMotor.getCurrentPosition() + deltaDeg * degsPerClick;

        // Update motor
        turretMotor.setTargetPosition((int) turretTargetGoal);

        // Telemetry
        telemetry.addData("Target Angle", targetAngleDeg);
        telemetry.addData("Turret Current Angle", turretCurrentDeg);
        telemetry.addData("Delta Deg", deltaDeg);
        telemetry.addData("Turret Target Clicks", turretTargetGoal);


    }
}
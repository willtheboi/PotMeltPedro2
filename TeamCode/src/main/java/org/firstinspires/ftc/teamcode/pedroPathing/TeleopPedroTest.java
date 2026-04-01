package org.firstinspires.ftc.teamcode.pedroPathing;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class TeleopPedroTest extends OpMode {

    DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    DcMotor intake;
    DcMotorEx launcher1, launcher2;
    Servo hood;

    CRServo wheel;



    /*public static double integralSumL = 0;
    public static double lastErrorL = 0;

    public static ElapsedTime timerL = new ElapsedTime();

    public static double Kp = 0.07;
    public static double Ki = 0.02;
    public static double Kd = 0.01;*/
    private boolean relocalizeToggle = false;
    private boolean relocalizeButtonToggle = true;
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
        hood = hardwareMap.get(Servo.class, "hood");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0); // Switch to pipeline number 0

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
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



        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");
        telemetry.addData("Launcher Speed", launcher1.getVelocity());
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

        if(gamepad1.a){
            timeSinceLocalized = time;

            if(timeSinceLocalized - time > 5000){
                relocalizeButtonToggle = true;
            }
        }

        if(relocalizeButtonToggle){
            follower.setPose(botCameraPose);
            relocalizeButtonToggle = false;
        }

        telemetry.addData("PedroFollower: ", follower.getPose());
        telemetry.update();

        relocalizationUpdate(limelight, telemetry);

        launcher1.setVelocity(gamepad2.left_trigger*-1550);
        launcher2.setVelocity(gamepad2.left_trigger*-1550);

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
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

                botpose2D = new Pose2D(DistanceUnit.INCH, result.getBotpose().getPosition().x * 39.37008, result.getBotpose().getPosition().y * 39.37008, AngleUnit.RADIANS, follower.getHeading());
                botPoseAsPedro = getFTCPoseAsPedro(botpose2D);

                telemetry.addData("Limelight Coordinates As Pedro: ", getFTCPoseAsPedro(botpose2D));

                botCameraPose = new Pose(botPoseAsPedro.getX(), botPoseAsPedro.getY(), follower.getHeading());
            } else {
                relocalizeToggle = false;
            }

        }
    }
}
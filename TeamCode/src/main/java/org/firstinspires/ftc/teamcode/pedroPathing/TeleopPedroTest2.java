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
public class TeleopPedroTest2 extends OpMode {

    DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    DcMotor intake;
    DcMotorEx launcher1, launcher2, turretMotor;
    Servo hood;
    CRServo wheel;

    public static ElapsedTime timer = new ElapsedTime();

    private boolean relocalizeToggle = false;
    private boolean relocalizeButtonToggle = false;
    double timeSinceLocalized = 0;

    Limelight3A limelight;
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    // Turret constants — corrected naming
    public static final double TICKS_PER_REVOLUTION = 1445.58;
    public static final double CLICKS_PER_DEG = TICKS_PER_REVOLUTION / 360.0; // ticks per degree
    public static final double DEGS_PER_CLICK = 360.0 / TICKS_PER_REVOLUTION; // degrees per tick
    public static final double MAX_TURRET_DEGREES = 298.0;                     // 1197 ticks / CLICKS_PER_DEG

    // Target poses
    public Pose targetPoseBlue = new Pose(5, 142);
    public Pose targetPoseRed = new Pose(140, 140);

    double turretTargetGoal = 0;
    Pose botCameraPose;
    boolean opModeIsStarted = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder()
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
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Turret setup — power set once here, never again in turretMovement
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.3);
    }

    @Override
    public void start() {
        opModeIsStarted = true;
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        // Drive control
        if (!automatedDrive) {
            if (!slowMode) {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        true
                );
            } else {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * slowModeMultiplier,
                        -gamepad1.left_stick_x * slowModeMultiplier,
                        -gamepad1.right_stick_x * slowModeMultiplier,
                        true
                );
            }
        }

        // Intake control
        if (gamepad2.left_bumper) {
            intake.setPower(-1);
            wheel.setPower(1);
        } else if (gamepad2.right_bumper) {
            intake.setPower(1);
            wheel.setPower(-1);
        } else {
            intake.setPower(0);
            wheel.setPower(0);
        }

        // Hood control
        if (gamepad2.dpad_up) {
            hood.setPosition(0.58);
        } else if (gamepad2.dpad_down) {
            hood.setPosition(0.73);
        }

        // Relocalization
        relocalizationUpdate(limelight, telemetry);

        // Manual relocalize on A
        LLResult result = limelight.getLatestResult();
        if (gamepad1.a && result != null && result.isValid()) {
            follower.setPose(botCameraPose);
        }

        // Turret tracking — absolute positioning, no drift
        turretMovement(true);

        // Launcher
        launcher1.setVelocity(gamepad2.left_trigger * -1550);
        launcher2.setVelocity(gamepad2.left_trigger * -1550);

        // Telemetry
        telemetry.addData("Pedro Pose: ", follower.getPose());
        telemetry.addData("Turret Current Ticks: ", turretMotor.getCurrentPosition());
        telemetry.update();
    }

    private Pose getFTCPoseAsPedro(Pose2D ftcPose) {
        return new Pose(
                ftcPose.getY(DistanceUnit.INCH) + 72,
                Math.abs(ftcPose.getX(DistanceUnit.INCH) - 72),
                follower.getHeading()
        );
    }

    public void relocalizationUpdate(Limelight3A limelight, Telemetry telemetry) {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose2D botpose2D = new Pose2D(
                    DistanceUnit.INCH,
                    result.getBotpose().getPosition().x * 39.37008,
                    result.getBotpose().getPosition().y * 39.37008,
                    AngleUnit.RADIANS,
                    follower.getHeading()
            );

            Pose botPoseAsPedro = getFTCPoseAsPedro(botpose2D);
            botCameraPose = new Pose(botPoseAsPedro.getX(), botPoseAsPedro.getY(), follower.getHeading());

            telemetry.addData("Limelight Pose As Pedro: ", botCameraPose);
        }
    }

    public void turretMovement(boolean isRed) {
        Pose targetPosition = isRed ? targetPoseRed : targetPoseBlue;

        // Field-space angle from robot to target (radians, then degrees)
        double angleToTargetDeg = Math.toDegrees(Math.atan2(
                targetPosition.getY() - follower.getPose().getY(),
                targetPosition.getX() - follower.getPose().getX()
        ));

        // Pedro heading is radians, CCW positive — convert to degrees
        double robotHeadingDeg = Math.toDegrees(follower.getPose().getHeading());

        // How far turret must rotate relative to robot forward
        double turretAngleDeg = angleToTargetDeg - robotHeadingDeg;

        // Normalize to [-180, 180] to always take the shortest path
        turretAngleDeg = ((turretAngleDeg % 360) + 360) % 360;
        if (turretAngleDeg > 180) turretAngleDeg -= 360;

        // Clamp to physical limits (±1197 ticks = ±298 degrees)
        turretAngleDeg = Math.max(-MAX_TURRET_DEGREES, Math.min(MAX_TURRET_DEGREES, turretAngleDeg));

        // Absolute tick target — no drift, no accumulation
        turretTargetGoal = turretAngleDeg * CLICKS_PER_DEG;
        turretMotor.setTargetPosition((int) turretTargetGoal);

        telemetry.addData("Angle To Target (deg): ", angleToTargetDeg);
        telemetry.addData("Robot Heading (deg): ", robotHeadingDeg);
        telemetry.addData("Turret Angle (deg): ", turretAngleDeg);
        telemetry.addData("Turret Target Ticks: ", turretTargetGoal);
    }
}
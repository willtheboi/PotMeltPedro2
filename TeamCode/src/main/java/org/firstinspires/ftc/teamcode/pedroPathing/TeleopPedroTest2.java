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

    Limelight3A limelight;
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    // Turret constants — update 1800 to your actual CPR if needed
    public double degsPerClick   = 360d  / 1445d; // degrees per encoder click
    public double clicksPerDeg   = 1445d / 360d;  // encoder clicks per degree
    // Hard limits in clicks (±1197 from your old code, adjust if needed)
    private static final double TURRET_MAX_CLICKS = 1197;

    public Pose targetPoseBlue = new Pose(5, 142);
    public Pose targetPoseRed  = new Pose(142, 138);

    double turretTargetGoal = 0;

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

        intake      = hardwareMap.get(DcMotor.class,   "intake");
        wheel       = hardwareMap.get(CRServo.class,   "wheel");
        launcher1   = hardwareMap.get(DcMotorEx.class, "launcher1");
        launcher2   = hardwareMap.get(DcMotorEx.class, "launcher2");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        hood        = hardwareMap.get(Servo.class,     "hood");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setPower(0.5);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

        // ── Drivetrain ──────────────────────────────────────────────
        if (!automatedDrive) {
            double scale = slowMode ? slowModeMultiplier : 1.0;
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y  * scale,
                    -gamepad1.left_stick_x  * scale,
                    -gamepad1.right_stick_x * scale,
                    true // robot-centric
            );
        }

        // ── Intake ───────────────────────────────────────────────────
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

        // ── Hood ─────────────────────────────────────────────────────
        if      (gamepad2.dpad_up)   hood.setPosition(0.58);
        else if (gamepad2.dpad_down) hood.setPosition(0.73);

        // ── Limelight relocalization ──────────────────────────────────
        relocalizationUpdate(limelight, telemetry);

        LLResult result = limelight.getLatestResult();
        if (gamepad1.a && result != null && result.isValid() && botCameraPose != null) {
            follower.setPose(botCameraPose);
        }

        // ── Turret ───────────────────────────────────────────────────
        if (opModeIsStarted) {
            turretMovement();
            telemetry.addData("Turret Clicks", turretMotor.getCurrentPosition());
        }

        // ── Launchers ────────────────────────────────────────────────
        launcher1.setVelocity(gamepad2.left_trigger * -1550);
        launcher2.setVelocity(gamepad2.left_trigger * -1550);

        telemetry.addData("Pedro Pose", follower.getPose());
        telemetry.update();
    }

    // ── Turret tracking ───────────────────────────────────────────────
    public void turretMovement() {
        Pose robotPose      = follower.getPose();
        double robotHeadingDeg = Math.toDegrees(robotPose.getHeading()); // 0–360

        // Current absolute turret angle on the field
        double turretCurrentDeg = turretMotor.getCurrentPosition() * degsPerClick + robotHeadingDeg;

        // Angle from robot to each goal
        double angleToRed = Math.toDegrees(Math.atan2(
                targetPoseRed.getY()  - robotPose.getY(),
                targetPoseRed.getX()  - robotPose.getX()
        ));
        double angleToBlue = Math.toDegrees(Math.atan2(
                targetPoseBlue.getY() - robotPose.getY(),
                targetPoseBlue.getX() - robotPose.getX()
        ));

        // Pick whichever goal the robot is facing
        double deltaToRed  = Math.abs(normalizeAngle(angleToRed  - robotHeadingDeg));
        double deltaToBlue = Math.abs(normalizeAngle(angleToBlue - robotHeadingDeg));
        boolean trackingRed = deltaToRed <= deltaToBlue;

        double targetAngleDeg = trackingRed ? angleToRed : angleToBlue;

        // How far does the turret need to rotate?
        double deltaDeg = normalizeAngle(targetAngleDeg - turretCurrentDeg);

        // Convert to clicks and clamp to physical limits
        double rawTarget = turretMotor.getCurrentPosition() + deltaDeg * clicksPerDeg;
        turretTargetGoal = Math.max(-TURRET_MAX_CLICKS, Math.min(TURRET_MAX_CLICKS, rawTarget));

        turretMotor.setTargetPosition((int) turretTargetGoal);

        telemetry.addData("Tracking",             trackingRed ? "RED" : "BLUE");
        telemetry.addData("Target Angle (deg)",   targetAngleDeg);
        telemetry.addData("Turret Angle (deg)",   turretCurrentDeg);
        telemetry.addData("Delta (deg)",          deltaDeg);
        telemetry.addData("Turret Target Clicks", turretTargetGoal);
    }

    // ── Helpers ───────────────────────────────────────────────────────
    private double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle >  180) return angle - 360;
        if (angle < -180) return angle + 360;
        return angle;
    }

    Pose botCameraPose;
    public void relocalizationUpdate(Limelight3A limelight, Telemetry telemetry) {
        limelight.updateRobotOrientation(Math.toDegrees(follower.getHeading()));
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid() && result.getFiducialResults().size() >= 1) {
            double x = result.getBotpose_MT2().getPosition().x * 39.37008;
            double y = result.getBotpose_MT2().getPosition().y * 39.37008;

            Pose2D botpose2D = new Pose2D(DistanceUnit.INCH, x, y,
                    AngleUnit.RADIANS, follower.getHeading());

            Pose botPoseAsPedro = getFTCPoseAsPedro(botpose2D);
            botCameraPose = new Pose(botPoseAsPedro.getX(), botPoseAsPedro.getY(), follower.getHeading());
            telemetry.addData("Limelight Pose (Pedro)", botCameraPose);
        }
    }

    private Pose getFTCPoseAsPedro(Pose2D ftcPose) {
        return new Pose(
                ftcPose.getY(DistanceUnit.INCH) + 72,
                Math.abs(ftcPose.getX(DistanceUnit.INCH) - 72),
                follower.getHeading()
        );
    }
}
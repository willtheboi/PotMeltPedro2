package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(name = "Turret Encoder Calibration", group = "Calibration")
public class EncoderCount extends OpMode {

    DcMotorEx turretMotor;

    double startPos = 0;
    double endPos = 0;

    boolean startCaptured = false;
    boolean endCaptured = false;
    double knownAngle = 90;

    @Override
    public void init() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");

        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
    @Override
    public void loop() {

        // Manual control with joystick
        double power = -gamepad1.left_stick_x;
        turretMotor.setPower(power * 0.4); // slow it down for precision

        // Capture START position
        if (gamepad1.a && !startCaptured) {
            startPos = turretMotor.getCurrentPosition();
            startCaptured = true;
        }

        // Capture END position
        if (gamepad1.b && startCaptured && !endCaptured) {
            endPos = turretMotor.getCurrentPosition();
            endCaptured = true;
        }
        if (gamepad1.x) {
            startCaptured = false;
            endCaptured = false;
        }
        double delta = endPos - startPos;
        double clicksPerDegree = (endCaptured) ? delta / knownAngle : 0;

        telemetry.addData("Current Encoder", turretMotor.getCurrentPosition());
        telemetry.addData("Start Pos", startPos);
        telemetry.addData("End Pos", endPos);
        telemetry.addData("Delta Clicks", delta);
        telemetry.addData("Known Angle", knownAngle);
        telemetry.addData("Clicks per Degree", clicksPerDegree);
        telemetry.update();
    }
}



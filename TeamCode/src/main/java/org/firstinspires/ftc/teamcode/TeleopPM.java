
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


    @TeleOp(name = "Teleop Potential Meltdown", group = "Robot")
//@Disabled
    public class TeleopPM extends OpMode {
        // This declares the motors needed
        DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
        DcMotor intakeL, intakeR;
        DcMotor outake1, outake2;
        DcMotor launcherL, launcherR;
        CRServo feederL, feederR;
        Servo servo;


        @Override
        public void init() {
            frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
            frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
            backLeftDrive = hardwareMap.get(DcMotor.class, "leftRear");
            backRightDrive = hardwareMap.get(DcMotor.class, "rightRear");
            intakeL = hardwareMap.get(DcMotor.class, "intakeL");
            intakeR = hardwareMap.get(DcMotor.class, "intakeR");
            //outake1 = hardwareMap.get(DcMotor.class, "outake1");
            //outake2 = hardwareMap.get(DcMotor.class, "outake2");
            //launcherL = hardwareMap.get(DcMotor.class, "launcherL");
            //launcherR = hardwareMap.get(DcMotor.class, "launcherR");
            //feederL = hardwareMap.get(CRServo.class, "feeder1");
            //feederR = hardwareMap.get(CRServo.class, "feeder2");
            //servo = hardwareMap.get(Servo.class, "servo"); // port 0 Control Hub


            // We set the left motors in reverse which is needed for drive trains where the left
            // motors are opposite to the right ones.
            backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            intakeL.setDirection(DcMotor.Direction.REVERSE);
            backRightDrive.setDirection(DcMotor.Direction.FORWARD);
            frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
            intakeR.setDirection(DcMotor.Direction.FORWARD);

            // outake2.setDirection(DcMotor.Direction.REVERSE);
            //launcherL.setDirection(DcMotor.Direction.REVERSE);


            // This uses RUN_USING_ENCODER to be more accurate.
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            // Set all drive motors to brake when power is zero
            frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Set intake motors to brake as well (optional, if you want them to stop more forcefully)
            intakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        }

        @Override
        public void loop() {

            telemetry.addLine("The left joystick sets the robot direction");
            telemetry.addLine("Moving the right joystick left and right turns the robot");
            //telemetry.addData("Servo Position", servo.getPosition());
            //telemetry.addData("Servo Speed", feederL.getPower());
            //telemetry.addData("Servo Speed", feederR.getPower());
            telemetry.update();


        /*if (gamepad1.a) {
            servo.setPosition(1);

        }else if (gamepad1.b){
            servo.setPosition(0);

        }else if (gamepad1.y){
            servo.setPosition (0.5);


    }
    */
            //intake of left and right bumper)
            if (gamepad1.left_bumper) {
                intakeL.setPower(-1);
                intakeR.setPower(-1);
                //feederL.setPower(1);
                //feederR.setPower(1);


                // adjust numbers to work where they put the intake on the robot
            } else if (gamepad1.right_bumper) {
                intakeL.setPower(1);
                intakeR.setPower(1);
                //feederL.setPower(-1);
                //feederR.setPower(-1);


            } else {
                intakeL.setPower(0);
                intakeR.setPower(0);
                //feederL.setPower(0);
                //feederR.setPower(0);


            }


            // launcher
            if (gamepad1.left_bumper) {
                //outake1.setPower(1);
                //outake2.setPower(1);


                // adjust numbers to work where they put the intake on the robot
            } else if (gamepad1.right_bumper) {
                //outake1.setPower(-1);
                //outake2.setPower(-1);


            } else {
                //outake1.setPower(0);
                //outake2.setPower(0);
            }


            double strafe = gamepad1.right_trigger - gamepad1.left_trigger;

            drive(-gamepad1.left_stick_y, strafe, gamepad1.right_stick_x);

        }


        public void drive(double forward, double right, double rotate) {
            // This calculates the power needed for each wheel based on the amount of forward,
            // strafe right, and rotate
            double frontLeftPower = forward + right + rotate;
            double frontRightPower = forward - right - rotate;
            double backRightPower = forward + right - rotate;
            double backLeftPower = forward - right + rotate;

            double maxPower = 1.0;
            double maxSpeed = 1.0;  // make this slower for outreaches

            // This is needed to make sure we don't pass > 1.0 to any wheel
            // It allows us to keep all of the motors in proportion to what they should
            // be and not get clipped
            maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
            maxPower = Math.max(maxPower, Math.abs(frontRightPower));
            maxPower = Math.max(maxPower, Math.abs(backRightPower));
            maxPower = Math.max(maxPower, Math.abs(backLeftPower));

            // We multiply by maxSpeed so that it can be set lower for outreaches
            // may not want to allow full

            frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
            frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
            backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
            backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));

        }
    }

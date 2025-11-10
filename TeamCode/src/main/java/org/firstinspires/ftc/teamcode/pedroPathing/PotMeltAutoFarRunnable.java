package org.firstinspires.ftc.teamcode.pedroPathing;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "PotMelt Auto Far (Runnable)", group = "Autonomous")
public class PotMeltAutoFarRunnable extends PotMeltAutoFar {

    @Override
    protected void autonomousPathUpdate() {
        // Called each time the path state changes.
        telemetry.addData("autonomousPathUpdate", "Path state: " + getPathState());
        telemetry.update();

        // Example: Add logic here if you want to control arms, intake, etc.

        /*switch (getPathState()) {
            case 1:
                //launcherL.setPower(1.0);
                SystemClock.sleep(200);
            case 3:
                launcherL.setPower(0);
                break;
        }*/

    }
}
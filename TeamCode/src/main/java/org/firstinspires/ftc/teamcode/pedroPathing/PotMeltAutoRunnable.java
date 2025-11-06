package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "PotMelt Auto (Runnable)", group = "Autonomous")
public class PotMeltAutoRunnable extends PotMeltAuto {

    @Override
    protected void autonomousPathUpdate() {
        // Called each time the path state changes.
        telemetry.addData("autonomousPathUpdate", "Path state: " + getPathState());
        telemetry.update();

        // Example: Add logic here if you want to control arms, intake, etc.

        switch (getPathState()) {
            case 0:
                // Intake on
                //follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                // Intake off

                break;
        }

    }
}
package org.firstinspires.ftc.teamcode.temp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by USER on 11/3/2017.
 */

@Autonomous(name="Temp Auto RED", group ="Temp")
public class TempAutoBlue extends TempOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

//        initVuforia();

        waitForStart();

        cubeLinearArm.setPower(-0.5);

        sleep(400);

        cubeLinearArm.setPower(0);

        pushBall(BLUE);
    }
}
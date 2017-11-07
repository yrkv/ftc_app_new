package org.firstinspires.ftc.teamcode.temp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by USER on 11/3/2017.
 */

@Autonomous(name="Temp Auto BLUE", group ="Temp")
public class TempAutoRed extends TempOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

//        initVuforia();

        waitForStart();

        cubeLinearArm.setPower(-0.5);

        sleep(400);

        cubeLinearArm.setPower(0);

        boolean run = false;
        boolean which = false;

        for (double pos = 0.2; pos < 1; pos += 0.01) {
            ballPoosher.setPosition(pos);
            sleep(10);

            if (colorSensor.red() > 3 || colorSensor.blue() > 3) {
                run = true;
                which = colorSensor.red() > colorSensor.blue();
                break;
            }
        }

        if (run) {
            if (which) {
                ghettoTurn(-0.25);
            } else {
                ghettoTurn(0.25);
            }

            ballPoosher.setPosition(1);
        }
    }
}
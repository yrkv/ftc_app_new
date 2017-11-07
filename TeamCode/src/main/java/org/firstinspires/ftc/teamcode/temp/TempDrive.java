package org.firstinspires.ftc.teamcode.temp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Button;
import org.firstinspires.ftc.teamcode.ButtonEvent;

/**
 * Created by USER on 10/13/2017.
 */

@TeleOp(name="tempDrive", group="Temp")
public class TempDrive extends TempOpMode {

    private int reverse = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

//        initVuforia();

        addButtonEvent(2, new ButtonEvent(Button.A) {
            private boolean grabbing = true;

            void onDown() {
                grabber1.setPosition(grabbing ? 1 : 0);
                grabber2.setPosition(grabbing ? 1 : 0);
                grabbing = !grabbing;
            }
        });

        addButtonEvent(1, new ButtonEvent(Button.X) {
            void onDown() {
                reverse *= -1;
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_trigger > 0.5)
                drive(0.2 * reverse);
            else if (gamepad1.right_trigger > 0.5)
                driveForward(0.3);
            else
                drive(0.4 * reverse);

            double armPower = gamepad2.dpad_up ?  -0.5 : (gamepad2.dpad_down ?  0.5 : 0);


            cubeLinearArm.setPower(armPower);


            runButtonEvents();

            telemetry.update();
        }
    }

    private void drive(double mult) {
        leftFront.setPower(mult * gamepad1.left_stick_y);
        leftBack.setPower(mult * gamepad1.left_stick_y);
        rightFront.setPower(mult * gamepad1.right_stick_y);
        rightBack.setPower(mult * gamepad1.right_stick_y);
    }

    private void driveForward(double mult) {
        mult *= -1;
        leftFront.setPower(mult);
        leftBack.setPower(mult);
        rightFront.setPower(mult);
        rightBack.setPower(mult);
    }
}

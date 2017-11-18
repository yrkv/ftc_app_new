package org.firstinspires.ftc.teamcode.temp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Button;
import org.firstinspires.ftc.teamcode.ButtonEvent;
import org.firstinspires.ftc.teamcode.Motor8696;

/**
 * Created by USER on 10/13/2017.
 */

@TeleOp(name="tempDrive", group="Temp")
public class TempDrive extends TempOpMode {

    private int reverse = 1;
    private static final double DPAD_MULT = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

//        initVuforia();


        addButtonEvent(2, new ButtonEvent(Button.A) {
            private boolean grabbing = true;

            public void onDown() {
                grabber1.setPosition(grabbing ? 1 : 0);
                grabber2.setPosition(grabbing ? 1 : 0);
                grabbing = !grabbing;
            }
        });

        addButtonEvent(1, new ButtonEvent(Button.X) {
            public void onDown() {
                reverse *= -1;
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_right) {
                leftBack.setPower(DPAD_MULT);
                rightBack.setPower(-DPAD_MULT);
                leftFront.setPower(-DPAD_MULT);
                rightFront.setPower(DPAD_MULT);
            } else if (gamepad1.dpad_left) {
                leftBack.setPower(-DPAD_MULT);
                rightBack.setPower(DPAD_MULT);
                leftFront.setPower(DPAD_MULT);
                rightFront.setPower(-DPAD_MULT);
            } else if (gamepad1.dpad_up) {
                driveForward(DPAD_MULT);
            } else if (gamepad1.dpad_down) {
                driveForward(-DPAD_MULT);
            } else {
                driveSideways(reverse);
                drive(reverse);

                for (Motor8696 motor : motors) {
                    motor.setMaxPower(Math.sqrt(2));
                }

                runMotors();
            }


            double armPower = gamepad2.dpad_up ?  -0.5 : (gamepad2.dpad_down ?  0.5 : 0);


            for (Motor8696 motor : motors) {
                telemetry.addData(motor.getDeviceName(), motor.getCurrentPosition());
            }

            cubeLinearArm.setPower(armPower);

            runButtonEvents();

            telemetry.update();
        }
    }

    private void drive(double mult) {
//        leftFront.setPower(mult * gamepad1.left_stick_y);
//        leftBack.setPower(mult * gamepad1.left_stick_y);
//        rightFront.setPower(mult * gamepad1.right_stick_y);
//        rightBack.setPower(mult * gamepad1.right_stick_y);

        leftFront.addPower(mult * gamepad1.left_stick_y);
        leftBack.addPower(mult * gamepad1.left_stick_y);
        rightFront.addPower(mult * gamepad1.right_stick_y);
        rightBack.addPower(mult * gamepad1.right_stick_y);
    }

    private void driveSideways(double mult) {
        leftFront.addPower(mult * -(gamepad1.right_trigger - gamepad1.left_trigger));
        leftBack.addPower(mult * (gamepad1.right_trigger - gamepad1.left_trigger));
        rightFront.addPower(mult * (gamepad1.right_trigger - gamepad1.left_trigger));
        rightBack.addPower(mult * -(gamepad1.right_trigger - gamepad1.left_trigger));
    }

    private void driveForward(double mult) {
        mult *= -1;
        leftFront.setPower(mult);
        leftBack.setPower(mult);
        rightFront.setPower(mult);
        rightBack.setPower(mult);
    }
}

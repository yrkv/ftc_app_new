package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by USER on 10/13/2017.
 */

@TeleOp(name="tempDrive", group="Linear Opmode")
public class TempDrive extends OpMode8696 {
    DcMotor cubeLinearArm;
    Servo grabber;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        cubeLinearArm = hardwareMap.get(DcMotor.class, "cubeArm");
        grabber = hardwareMap.get(Servo.class, "grabber");

        cubeLinearArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        initVuforia();

        // I have no idea if this actually works
        addButtonEvent(new ButtonEvent(BUTTON.A) {
            private double grabbing = 1;
            private double notGrabbing = 0.7;

            void onDown() {
                double pos = grabber.getPosition();
                grabber.setPosition((pos == grabbing) ? notGrabbing : grabbing);
            }
        });

        waitForStart();

//        boolean servo = false;
//        boolean wasA = false;

        while (opModeIsActive()) {
            if (gamepad1.left_trigger > 0.5)
                drive(0.75);
            else if (gamepad1.right_trigger > 0.5)
                driveForward(0.3);
            else
                drive(0.4);

            double armPower = gamepad2.dpad_up ?  -0.5 : (gamepad2.dpad_down ?  0.5 : 0);

            cubeLinearArm.setPower(armPower);

//            if (gamepad2.a) {
//                if (!wasA) {
//                    grabber.setPosition(servo ? 0.7 : 1); // not grabbing to grabbing
//                    servo = !servo;
//                }
//                wasA = true;
//            } else
//                wasA = false;

            runButtonEvents();
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

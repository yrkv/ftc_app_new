package org.firstinspires.ftc.teamcode.temp;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Motor8696;
import org.firstinspires.ftc.teamcode.OpMode8696;

/**
 * Created by USER on 11/5/2017.
 */

public abstract class TempOpMode extends OpMode8696 {
    DcMotor cubeLinearArm;
    Servo grabber1;
    Servo grabber2;
    Servo ballPoosher;

    ModernRoboticsI2cColorSensor colorSensor;

    void initRobot() {
        initMotors();

        leftBack  .setDirection(DcMotor.Direction.FORWARD);
        rightBack .setDirection(DcMotor.Direction.REVERSE);
        leftFront .setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        rightBack .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack  .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        cubeLinearArm = hardwareMap.get(DcMotor.class, "cubeArm");
        grabber1 = hardwareMap.get(Servo.class, "grabber1");
        grabber2 = hardwareMap.get(Servo.class, "grabber2");
        ballPoosher = hardwareMap.get(Servo.class, "ballPoosher");
        colorSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "color");

//        colorSensor.setI2cAddress(I2cAddr.create8bit(0x3a));

        grabber2.setDirection(Servo.Direction.REVERSE);

        grabber1.scaleRange(0.5, 0.7);
        grabber2.scaleRange(0.3, 0.5);

        ballPoosher.scaleRange(0.1, 1); //TODO: set correct values

        grabber1.setPosition(1);
        grabber2.setPosition(1);
        ballPoosher.setPosition(0);

        colorSensor.enableLed(false);
        colorSensor.enableLed(true);

        cubeLinearArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    void ghettoTurn(double power) {
        leftFront.setPower(-power);
        leftBack.setPower(-power);
        rightFront.setPower(power);
        rightBack.setPower(power);

        sleep(250);

        for (Motor8696 motor : motors)
            motor.setPower(0);
    }
}

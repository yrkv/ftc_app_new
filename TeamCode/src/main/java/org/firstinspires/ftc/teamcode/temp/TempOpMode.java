package org.firstinspires.ftc.teamcode.temp;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Motor8696;
import org.firstinspires.ftc.teamcode.OpMode8696;
import org.firstinspires.ftc.teamcode.RunUntil;

/**
 * Created by USER on 11/5/2017.
 */

public abstract class TempOpMode extends OpMode8696 {
    DcMotor cubeLinearArm;
    Servo grabber1;
    Servo grabber2;
    Servo ballPoosher;
    
//    ColorSensor tapeColorSensor;

    ColorSensor ballColorSensor;

    static final boolean RED = true;
    static final boolean BLUE = false;

    /**
     * constant for how many encoder counts are equivalent
     * to strafing one inch sideways.
     */
    private static final double STRAFE_COEFFICIENT = 400;

    void initRobot() {
        initMotors();

        leftBack  .setDirection(DcMotor.Direction.FORWARD);
        rightBack .setDirection(DcMotor.Direction.REVERSE);
        leftFront .setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        for (Motor8696 motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        cubeLinearArm = hardwareMap.get(DcMotor.class, "cubeArm");
        grabber1 = hardwareMap.get(Servo.class, "grabber1");
        grabber2 = hardwareMap.get(Servo.class, "grabber2");
        ballPoosher = hardwareMap.get(Servo.class, "ballPoosher");
        ballColorSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "color"); // TODO: change config name
//        tapeColorSensor = hardwareMap.get(ColorSensor.class, "tapeColor");

//        ballColorSensor.setI2cAddress(I2cAddr.create7bit(0x3c));

//        ballColorSensor.resetDeviceConfigurationForOpMode();

        telemetry.addData("color", ballColorSensor.getConnectionInfo());
        telemetry.update();

        grabber2.setDirection(Servo.Direction.REVERSE);

        grabber1.scaleRange(0.5, 0.8);
        grabber2.scaleRange(0.2, 0.5);

        ballPoosher.scaleRange(0.1, 1); //TODO: set correct values

        grabber1.setPosition(1);
        grabber2.setPosition(1);
        ballPoosher.setPosition(0);

        ballColorSensor.enableLed(false);
        ballColorSensor.enableLed(true);

        cubeLinearArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    void pushBall(boolean color) {
        boolean foundBall = false;
        boolean which = false;

        for (double pos = 0.2; pos < 1; pos += 0.01) {
            ballPoosher.setPosition(pos);
            sleep(10);

            if (ballColorSensor.red() > 3 || ballColorSensor.blue() > 3) {
                foundBall = true;
                which = ballColorSensor.red() > ballColorSensor.blue();
                break;
            }
        }

        if (foundBall) {
            if (which == color) {
                ghettoTurn(-0.25);
            } else {
                ghettoTurn(0.25);
            }

            ballPoosher.setPosition(1);

            tempAutoTurn(0, 0.25, 3);
        } else {
            telemetry.log().add("ball not found");
        }
    }

    void tempAutoTurn(double angle, double power, double timeoutSeconds) {
        runtime.reset();

        while (onHeading(angle, power, 0.5) &&
                runtime.seconds() < timeoutSeconds) {
            idle();
        }
    }

    private boolean onHeading(double angle, double power, double maxError) {
        getGyroData();

        double currentRotation = angles.firstAngle;

        double diff = angle - currentRotation;
        while (Math.abs(diff) >= 180) {
            angle += (diff >= 180) ? -180 : 180;
            diff = angle - currentRotation;
        }

        if (Math.abs(diff) > maxError) {
            power = 180 / diff * power;

            leftBack  .setPower( power);
            rightBack .setPower(-power);
            leftFront .setPower( power);
            rightFront.setPower(-power);
            return true;
        } else {
            return false;
        }
    }

    void tempAutoDrive(double inches, double power, double timeoutSeconds) {
        tempAutoDrive(inches, power, timeoutSeconds, new RunUntil() {
            @Override
            public boolean stop() {
                return false;
            }
        });
    }

    void tempAutoDrive(double inches, double power, double timeoutSeconds, RunUntil runUntil) {
        for (Motor8696 motor : motors) {
            motor.storePosition();
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motor.setRelativeTarget((int) (-inches / (4 * Math.PI) * Motor8696.COUNTS_PER_REVOLUTION));

            motor.setPower(power);
        }

        runtime.reset();

        while (opModeIsActive() &&
                runtime.seconds() < timeoutSeconds &&
                Motor8696.motorsBusy(motors) &&
                !runUntil.stop()) {
            idle();
        }

        for (Motor8696 motor : motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    void tempAutoStrafe(double inches, double power, double timeoutSeconds) {
        tempAutoStrafe(inches, power, timeoutSeconds, new RunUntil() {
            @Override
            public boolean stop() {
                return false;
            }
        });
    }

    /**
     * Strafe to the side a set distance
     *
     * @param inches Number of inches to strafe.
     *               positive is right, negative is left.
     * @param power Power to set the motors to.
     * @param timeoutSeconds After this many seconds, it stops trying.
     */
    void tempAutoStrafe(double inches, double power, double timeoutSeconds, RunUntil runUntil) {
        for (Motor8696 motor : motors) {
            motor.storePosition();
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        leftBack  .setTargetPosition((int) ( inches * STRAFE_COEFFICIENT));
        rightBack .setTargetPosition((int) (-inches * STRAFE_COEFFICIENT));
        leftFront .setTargetPosition((int) (-inches * STRAFE_COEFFICIENT));
        rightFront.setTargetPosition((int) ( inches * STRAFE_COEFFICIENT));

        for (Motor8696 motor : motors)
            motor.setPower(Math.abs(power));

        runtime.reset();

        while (opModeIsActive() &&
                runtime.seconds() < timeoutSeconds &&
                Motor8696.motorsBusy(motors) &&
                !runUntil.stop()) {
            idle();
        }

        for (Motor8696 motor : motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
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

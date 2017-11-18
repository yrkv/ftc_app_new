package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.OpMode8696;

/**
 * Created by USER on 11/5/2017.
 */

public abstract class MecanumOpMode extends OpMode8696 {

    protected void initRobot() {
        initMotors();

        leftBack  .setDirection(DcMotor.Direction.FORWARD);
        rightBack .setDirection(DcMotor.Direction.REVERSE);
        leftFront .setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        rightBack .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack  .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        navx = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = navx;

        telemetry.log().add("Gyro Calibrating. Do Not Move!");

//         Wait until the gyro calibration is complete
        int loop = 0;
        while (navx.isCalibrating())  {
            loop++;
            telemetry.addData(">", loop);
            telemetry.update();
            sleep(50);
            idle();
        }
        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();
    }


    /**
     * Calculate the angles for driving and move/rotate the robot.
     */
    protected void mecanumTeleOpDrive() {
        double ly = gamepad1.left_stick_y;
        double lx = -gamepad1.left_stick_x;
        double ry = gamepad1.right_stick_y;
        double rx = -gamepad1.right_stick_x;

        double leftStickAngle  = Math.atan(ly / lx);
        if (lx > 0) leftStickAngle += Math.PI;

        double rightStickAngle = Math.atan(ry / rx) - Math.PI / 2;
        if (rx > 0) rightStickAngle += Math.PI;

        getGyroData();

        if (!Double.isNaN(leftStickAngle) && getMagnitude(lx, ly) >= 0.25) {
            driveDirectionRelativeToRobot(leftStickAngle, getMagnitude(lx, ly));
        } else {
            driveDirectionRelativeToRobot(leftStickAngle, 0);
        }

        if (!Double.isNaN(rightStickAngle) && getMagnitude(rx, ry) >= 0.5) {
            rotateToAngleTeleOp(rightStickAngle);
        }

        telemetry.addData("rs", "%.2f", rightStickAngle * 180 / Math.PI);

        runMotors();
    }

    private double getMagnitude(double x, double y) {
        double magnitude = Math.sqrt(x*x + y*y);
        return Range.clip(magnitude, -1, 1);
    }

    /**
     * Drive the robot in a specified direction regardless of its
     * current rotation. Meant to be used with mecanum wheels.
     *
     * @param angle angle that the robot should move towards. Starts at standard position.
     * @param power number to scale all the motor power by.
     */
    private void driveDirectionRelativeToRobot(double angle, double power) {
        angle = angle - angles.firstAngle;
        leftBack  .addPower((Math.sin(angle) - Math.cos(angle)) * power, Math.sqrt(2));
        rightBack .addPower((Math.sin(angle) + Math.cos(angle)) * power, Math.sqrt(2));
        leftFront .addPower((Math.sin(angle) + Math.cos(angle)) * power, Math.sqrt(2));
        rightFront.addPower((Math.sin(angle) - Math.cos(angle)) * power, Math.sqrt(2));
    }

    /**
     * Rotate the robot a bit toward a specified angle.
     *
     * @param target the target angle
     */
    private void rotateToAngleTeleOp(double target) {
        double currentRotation = angles.firstAngle;

        double angle = target;

        double diff = angle - currentRotation;
        while (Math.abs(diff) >= Math.PI) {
            angle += (diff >= Math.PI) ? -Math.PI : Math.PI;
            diff = angle - currentRotation;
        }
        telemetry.addData("angles", "target:%.2f, angle:%.2f, robot:%.2f, diff:%.2f", target, angle, currentRotation, diff);

        if (Math.abs(diff) >= 2.5 * Math.PI / 180) {


            double power = diff / Math.PI;

            if (0 < power && power < 0.15) power = 0.15;
            if (-0.15 < power && power < 0) power = -0.15;

            leftBack.addPower(-power, 1);
            rightBack.addPower(power, 1);
            leftFront.addPower(-power, 1);
            rightFront.addPower(power, 1);
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/*
DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
        Version 2, December 2004

        Copyright (C) 2004 Sam Hocevar <sam@hocevar.net>

Everyone is permitted to copy and distribute verbatim or modified
copies of this license document, and changing it is allowed as long
as the name is changed.

DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION

0. You just DO WHAT THE FUCK YOU WANT TO.
*/

public abstract class OpMode8696 extends LinearOpMode {

    Motor8696 leftBack;
    Motor8696 rightBack;
    Motor8696 leftFront;
    Motor8696 rightFront;

    Motor8696[] motors = {leftBack, rightBack, leftFront, rightFront};

    private ButtonEvent[][] buttonEvents = new ButtonEvent[2][Button.values().length];

    private int[] wasPressed = new int[2];

    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;

    NavxMicroNavigationSensor navx;
    IntegratingGyroscope gyro;

    AngularVelocity rates;
    Orientation angles;


    protected void initRobot() {
        leftBack   = new Motor8696(hardwareMap.get(DcMotor.class, "leftBack"));
        rightBack  = new Motor8696(hardwareMap.get(DcMotor.class, "rightBack"));
        leftFront  = new Motor8696(hardwareMap.get(DcMotor.class, "leftFront"));
        rightFront = new Motor8696(hardwareMap.get(DcMotor.class, "rightFront"));

        navx = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = navx;

        telemetry.log().add("Gyro Calibrating. Do Not Move!");

        // Wait until the gyro calibration is complete
        int loop = 0;
        while (navx.isCalibrating())  {
            loop++;
            telemetry.addData(">", loop);
            telemetry.update();
            sleep(50);
        }
        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();

        leftBack  .setDirection(DcMotor.Direction.FORWARD);
        rightBack .setDirection(DcMotor.Direction.REVERSE);
        leftFront .setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        rightBack .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack  .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    protected void initVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AXsBBW7/////AAAAGXj7SVf450terrL5QOqPUr1Tozrj/sG57Z/tukLNECvwVhLUNaNxKv783tA6U2Kze0Hs+9EpVCJ8PzhKRCocFqWqDdZbqjktD2McMriGHUtCiIfoFyF5xKCZ11QBmMNTRBRkqV/s0HWgxkD41BA8d3ZlfS9zF7Vgh1397O35rqCY8KyjTqtaPzbxecZWb96/Bpq0Ct9u/e0e35d0+Vth/VdGp3vLMRFPNzPEZlJ6/VDQlgeHobmzJ7ccHKb6k7WPUC7vDyZEZXyIQPnAJoLbHT+j4kYFnVuFUaok5jrNn8TknXxpgRSvQTsxeilOQQtSxn/a9SNiR7pnpqRjLWAe0E1H5qu3a952fwo7PlGxkzk1";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        this.vuforia = new ClosableVuforiaLocalizer(parameters);

        vuforia.setFrameQueueCapacity(5);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

//        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
//        VuforiaTrackable relicTemplate = relicTrackables.get(0);

    }

    protected OpMode8696 addButtonEvent(int gamepad, ButtonEvent event) {
        buttonEvents[gamepad-1][event.button.ordinal()] = event;
        return this;
    }

    protected void runButtonEvents() {
        runButtonEvents(1);
        runButtonEvents(2);
    }

    private void runButtonEvents(int gamepad) {
        int currPressed = getButtonsPressed(gamepad);
        
        gamepad--;

        if (currPressed >= 0 && wasPressed[gamepad] >= 0) { // don't bother running if no buttons pressed
            // use bitwise operators to check all buttons at the same time
            int onDown    =  currPressed & ~wasPressed[gamepad];
            int onUp      = ~currPressed &  wasPressed[gamepad];
            int whileDown =  currPressed &  wasPressed[gamepad];
            int whileUp   = ~currPressed & ~wasPressed[gamepad];
            for (int i = 0; i < 15; i++) {
                ButtonEvent event = buttonEvents[gamepad][i];
                if (event != null) {
                    // select specific bits and check if the related event should be called.
                    if ((onDown    & (1 << i)) > 0) event.onDown();
                    if ((onUp      & (1 << i)) > 0) event.onUp();
                    if ((whileDown & (1 << i)) > 0) event.whileDown();
                    if ((whileUp   & (1 << i)) > 0) event.whileUp();
                }
            }
        }

        wasPressed[gamepad] = currPressed;
    }
    
    /**
     * Get the button data as an int from one of the gamepads.
     *
     * @param gamepad which gamepad to access.
     */
    
    private int getButtonsPressed(int gamepad) {
        int currPressed = 0;
        
        try {
            byte[] arr = ((gamepad == 1) ? gamepad1 : gamepad2).toByteArray(); // select the right gamepad
            int len = arr.length;

            // extract the button data from the byte array
            currPressed += arr[len-3]; currPressed = currPressed << 8;
            currPressed += arr[len-2] & 0b11111111;

            // the left-most bit of the byte is counted as the "negative" part,
            // and the sign is maintained when it becomes an int.
            // " & 0b11111111" limits it to the 8 bits I want.
            // This isn't an issue with the first part because there are only 7 used bits.
        } catch (RobotCoreException e) {
            e.printStackTrace();
        }
        
        return currPressed;
    }

    /**
     * Calculate the angles for driving and move/rotate the robot.
     */
    protected void mecanumTeleOpDrive() {
        double leftStickAngle  = Math.atan(gamepad1.left_stick_y  / gamepad1.left_stick_x);
        double rightStickAngle = Math.atan(gamepad1.right_stick_y / gamepad1.right_stick_x);

        for (int i = 0; i < motors.length; i++)
            motors[i].setScalePower(Math.sqrt(Math.pow(gamepad1.left_stick_y, 2)
                                            + Math.pow(gamepad1.left_stick_x, 2)));

        getGyroData();
        driveDirectionRelativeToRobot(leftStickAngle);
        rotateToAngleTeleOp(rightStickAngle);

        runMotors();
    }

    /**
     * Drive the robot in a specified direction regardless of its
     * current rotation. Meant to be used with mecanum wheels.
     *
     * @param angle angle that the robot should move towards. Starts at standard position.
     */
    private void driveDirectionRelativeToRobot(double angle) {
        angle = (angle - angles.firstAngle) * Math.PI / 180;
        leftBack  .addPower(Math.sin(angle) - Math.cos(angle), Math.sqrt(2));
        rightBack .addPower(Math.sin(angle) + Math.cos(angle), Math.sqrt(2));
        leftFront .addPower(Math.sin(angle) - Math.cos(angle), Math.sqrt(2));
        rightFront.addPower(Math.sin(angle) + Math.cos(angle), Math.sqrt(2));
    }

    /**
     * Rotate the robot a bit toward a specified angle.
     *
     * @param angle the target angle
     */
    private void rotateToAngleTeleOp(double angle) {
        double currentRotation = angles.firstAngle;

        double diff = angle - currentRotation;
        while (Math.abs(diff) >= 180) {
            angle += (diff >= 180) ? -180 : 180;
            diff = angle - currentRotation;
        }

        double power = 180 / diff;

        leftBack  .addPower( power, 1);
        rightBack .addPower(-power, 1);
        leftFront .addPower( power, 1);
        rightFront.addPower(-power, 1);
    }

    private void runMotors() {
        for (Motor8696 motor : motors) {
            motor.setPower();
        }
    }

    /**
     * Stores the gyro data into instance fields.
     */
    private void getGyroData() {
        rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
        angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
}

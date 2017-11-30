package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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

    protected Motor8696 leftBack;
    protected Motor8696 rightBack;
    protected Motor8696 leftFront;
    protected Motor8696 rightFront;

    protected ElapsedTime runtime = new ElapsedTime();

    protected Motor8696[] motors = new Motor8696[4];

    private ButtonEvent[][] buttonEvents = new ButtonEvent[2][Button.values().length];

    private int[] wasPressed = new int[2];

    protected VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;

    protected NavxMicroNavigationSensor navx;
    protected IntegratingGyroscope gyro;

    protected BNO055IMU imu;

    protected Acceleration gravity;
    protected Orientation angles;

    protected double driveSpeed = 1;

    public static final boolean RED = true;
    public static final boolean BLUE = false;

    /**
     * constant for how many encoder counts are equivalent
     * to strafing one inch sideways.
     */
    private static final double STRAFE_COEFFICIENT = 400; // TODO: make this value more exact

    /**
     * constant for how many encodes counts are equivalent
     * to rotating the robot one degree.
     */
    private static final double TURN_COEFFICIENT = 15;

    private RobotState robotState;

    private long lastPeriodicCall = 0;

    protected void periodic(long ms) {
        long t = System.currentTimeMillis();
        if (t >= lastPeriodicCall + ms) {
            lastPeriodicCall = t;
            periodic();
        }
    }

    protected void periodic() {

        robotState = new RobotState(this);
    }

    protected void initRobot() {
        initMotors();
        lastPeriodicCall = System.currentTimeMillis();
    }

    protected void initMotors() {
        leftFront  = new Motor8696(hardwareMap.get(DcMotor.class, "leftFront")); //0
        rightFront = new Motor8696(hardwareMap.get(DcMotor.class, "rightFront"));//1
        leftBack   = new Motor8696(hardwareMap.get(DcMotor.class, "leftBack"));  //2
        rightBack  = new Motor8696(hardwareMap.get(DcMotor.class, "rightBack")); //3

        motors[0] = leftBack;
        motors[1] = rightBack;
        motors[2] = leftFront;
        motors[3] = rightFront;
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

    protected void addButtonEvent(int gamepad, ButtonEvent event) {
        buttonEvents[gamepad-1][event.button.ordinal()] = event;
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
        double ly = gamepad1.left_stick_y;
        double lx = -gamepad1.left_stick_x;
        double ry = gamepad1.right_stick_y;
        double rx = -gamepad1.right_stick_x;

        double leftStickAngle  = Math.atan(ly / lx);
        if (lx > 0) leftStickAngle += Math.PI;

        double rightStickAngle = Math.atan(ry / rx) - Math.PI / 2;
        if (rx > 0) rightStickAngle += Math.PI;

        for (Motor8696 motor : motors) {
            motor.reset();
        }

        getGyroData();

        if (!Double.isNaN(leftStickAngle) && getMagnitude(lx, ly) >= 0.25) {
            driveDirectionRelativeToRobot(leftStickAngle, getMagnitude(lx, ly));
        }

        if (!Double.isNaN(rightStickAngle) && getMagnitude(rx, ry) >= 0.5) {
            onHeading(rightStickAngle * 180 / Math.PI, 0.5, 1, false);
            for (Motor8696 motor : motors)
                motor.addPower(0, 1);
        }

        runMotors();
    }

    protected double getMagnitude(double x, double y) {
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
        angle = adjustAngle(angle, angles.firstAngle);
        angle = angle / 180 * Math.PI;
        leftBack  .addPower((Math.sin(angle) - Math.cos(angle)) * power);
        rightBack .addPower((Math.sin(angle) + Math.cos(angle)) * power);
        leftFront .addPower((Math.sin(angle) + Math.cos(angle)) * power);
        rightFront.addPower((Math.sin(angle) - Math.cos(angle)) * power);
    }

    protected void autoTurn(double angle, double power, double timeoutSeconds) {
        runtime.reset();

        while (opModeIsActive() &&
                onHeading(angle, power, 0.5, true) &&
                runtime.seconds() < timeoutSeconds) {
            idle();
            runMotors();
            for (Motor8696 motor: motors) {
                motor.reset();
            }
        }
        runMotors();
    }

    protected boolean onHeading(double angle, double power, double maxError, boolean useEncoders) {
        getGyroData();

        double diff = adjustAngle(angle, angles.firstAngle);

        if (Math.abs(diff) > maxError) {
            if (useEncoders)
                encoderTurning(diff, power);
            else
                crappyTurning(diff, power);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Incrementally rotate the robot using the encoders, which makes use of the built in PID stuff.
     *
     * @param diff how much the robot needs to rotate to get to its target orientation.
     * @param power how much power to send to the motors.
     */
    private void encoderTurning(double diff, double power) {
        for (Motor8696 motor : motors) {
            motor.storePosition();
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        leftBack  .setRelativeTarget((int) (diff * TURN_COEFFICIENT));
        rightBack .setRelativeTarget((int)-(diff * TURN_COEFFICIENT));
        leftFront .setRelativeTarget((int) (diff * TURN_COEFFICIENT));
        rightFront.setRelativeTarget((int)-(diff * TURN_COEFFICIENT));

        for (Motor8696 motor : motors) {
            motor.addPower(power);
        }
    }

    /**
     * Incrementally rotate the robot using just the powers. Adjust the power as it gets close to the target.
     *
     * @param diff how much the robot needs to rotate to get to its target orientation.
     * @param power how much power to send to the motors.
     */
    private void crappyTurning(double diff, double power) {
        for (Motor8696 motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        double temp = diff/30 * power;
        if (Math.abs(temp) < Math.abs(power))
            power = temp;
        if (Math.abs(power) <= 0.15) {
            if (power < 0)
                power = -0.15;
            else
                power = 0.15;
        }

        leftBack  .addPower( power);
        rightBack .addPower(-power);
        leftFront .addPower( power);
        rightFront.addPower(-power);
    }

    /**
     * Calculate the difference between two angles and set the output to be in a range.
     * Used for calculating how far the robot needs to turn to get to a target rotation.
     *
     * @param target The target rotation
     * @param currentRotation The current robot orientation, (preferably) found by a gyro sensor.
     * @return Adjusted angle, -180 <= angle <= 180
     */
    private double adjustAngle(double target, double currentRotation) {
        double diff = target - currentRotation;
        while (Math.abs(diff) > 180) {
            target += (diff >= 180) ? -180 : 180;
            diff = target - currentRotation;
        }
        return diff;
    }

    protected void autoDrive(double inches, double power, double timeoutSeconds) {
        autoDrive(inches, power, timeoutSeconds, new RunUntil() {
            @Override
            public boolean stop() {
                return false;
            }
        });
    }

    void autoDrive(double inches, double power, double timeoutSeconds, RunUntil runUntil) {
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

    void autoStrafe(double inches, double power, double timeoutSeconds) {
        autoStrafe(inches, power, timeoutSeconds, new RunUntil() {
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
    void autoStrafe(double inches, double power, double timeoutSeconds, RunUntil runUntil) {
        for (Motor8696 motor : motors) {
            motor.storePosition();
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        leftBack  .setRelativeTarget((int) ( inches * STRAFE_COEFFICIENT));
        rightBack .setRelativeTarget((int) (-inches * STRAFE_COEFFICIENT));
        leftFront .setRelativeTarget((int) (-inches * STRAFE_COEFFICIENT));
        rightFront.setRelativeTarget((int) ( inches * STRAFE_COEFFICIENT));

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

    protected void runMotors() {
        for (Motor8696 motor : motors) {
            motor.setPower();
        }
    }

    /**
     * Stores the gyro data into instance fields.
     */
    protected void getGyroData() {
        gravity = imu.getGravity();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
}

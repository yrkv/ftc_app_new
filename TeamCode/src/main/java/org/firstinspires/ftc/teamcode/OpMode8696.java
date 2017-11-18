package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.opengl.models.MeshObject;

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

    AngularVelocity rates;
    protected Orientation angles;

    protected void initMotors() {
        leftBack   = new Motor8696(hardwareMap.get(DcMotor.class, "leftBack"));  //2
        rightBack  = new Motor8696(hardwareMap.get(DcMotor.class, "rightBack")); //3
        leftFront  = new Motor8696(hardwareMap.get(DcMotor.class, "leftFront")); //0
        rightFront = new Motor8696(hardwareMap.get(DcMotor.class, "rightFront"));//1

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

    protected void runMotors() {
        for (Motor8696 motor : motors) {
            motor.setPower();
        }
    }

    /**
     * Stores the gyro data into instance fields.
     */
    protected void getGyroData() {
        rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
        angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
    }
}

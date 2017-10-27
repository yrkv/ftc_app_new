package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.*;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Yegor on 10/4/2017.
 */

public abstract class OpMode8696 extends LinearOpMode {

    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor leftFront;
    DcMotor rightFront;

    private ButtonEvent[][] buttonEvents = new ButtonEvent[2][Button.values().length];

    private int[] wasPressed = new int[2];

    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;

    protected void initRobot() {
          leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
          rightBack    = hardwareMap.get(DcMotor.class, "rightBack");
          leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
          rightFront   = hardwareMap.get(DcMotor.class, "rightFront");

          leftBack  .setDirection(DcMotorSimple.Direction.FORWARD);
          rightBack .setDirection(DcMotorSimple.Direction.REVERSE);
          leftFront .setDirection(DcMotorSimple.Direction.FORWARD);
          rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
  
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

        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

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
            int onDown    =  currPressed & ~wasPressed[gamepad];
            int onUp      = ~currPressed &  wasPressed[gamepad];
            int whileDown =  currPressed &  wasPressed[gamepad];
            int whileUp   = ~currPressed & ~wasPressed[gamepad];
            for (int i = 0; i < 15; i++) {
                ButtonEvent event = buttonEvents[gamepad][i];
                if (event != null) {
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
     * Get the button data from one of the gamepads as an int.
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
     * Run a motor to a position relative to its current position
     *
     * @param encoderCounts number of encoder counts to set each target to.
     * @param speeds speeds to run the motors at
     * @param motors references to which motors to run
     */
    
    protected void encoderDrive(int[] encoderCounts, double[] speeds, DcMotor[] motors) {
        // TODO: do this.
    }

    /**
     * Drive the robot in a specified direction regardless of its current rotation. Meant to be used with mecanum wheels.
     *
     * @param angle angle that the robot should move towards. Starts at standard position.
     */
    protected void driveDirectionRelativeToRobot(double angle) {
        //TODO: do this.
    }
}

package org.firstinspires.ftc.teamcode;

import android.widget.Button;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.sun.tools.javac.code.Attribute;
import com.vuforia.*;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;

/**
 * Created by USER on 10/4/2017.
 */

public abstract class OpMode8696 extends LinearOpMode {

    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor leftFront;
    DcMotor rightFront;

    private ButtonEvent[] buttonEvents = new ButtonEvent[BUTTON.values().length];

    private int wasPressed = 0;

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

        vuforia.setFrameQueueCapacity(1);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

//        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
//        VuforiaTrackable relicTemplate = relicTrackables.get(0);

    }

    protected OpMode8696 addButtonEvent(ButtonEvent event) {
        buttonEvents[event.button.ordinal()] = event;
        return this;
    }

    // I have no idea if this works
    protected void runButtonEvents() {
        int currPressed = -1;

        try {
            byte[] arr = gamepad1.toByteArray();
            int len = arr.length;
            currPressed = 0;
            currPressed += arr[len-4]; currPressed = currPressed << 8;
            currPressed += arr[len-3]; currPressed = currPressed << 8;
            currPressed += arr[len-2]; currPressed = currPressed << 8;
            currPressed += arr[len-1];
        } catch (RobotCoreException e) {
            e.printStackTrace();
        }

        if (currPressed >= 0 && wasPressed >= 0) {
            int bools1 = currPressed & ~wasPressed;
            int bools2 = ~currPressed & wasPressed;
            int bools3 = currPressed & wasPressed;
            int bools4 = ~currPressed & ~wasPressed;
            for (int i = 0; i < 15; i++) {
                if (buttonEvents[i] != null) {
                    if ((bools1 & (1 << i)) > 0)
                        buttonEvents[i].onDown();
                    if ((bools2 & (1 << i)) > 0)
                        buttonEvents[i].onUp();
                    if ((bools3 & (1 << i)) > 0)
                        buttonEvents[i].whileDown();
                    if ((bools4 & (1 << i)) > 0)
                        buttonEvents[i].whileUp();
                }
            }
        }

        wasPressed = currPressed;
    }

    /**
     * @param angle angle that the robot should move at.
     *              Starts at standard position.
     */
    protected void driveDirectionRelativeToRobot(double angle) {
        //TODO: do this. Can't do it now since I don't have a robot to test with.
    }
}

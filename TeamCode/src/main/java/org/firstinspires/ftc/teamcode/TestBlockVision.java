package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.*;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.nio.ByteBuffer;

/**
 * Created by USER on 10/4/2017.
 */

@Autonomous(name="Testing Vision", group ="Test")
public class TestBlockVision extends OpMode8696 {


    @Override
    public void runOpMode() throws InterruptedException {
        initVuforia();

        telemetry.addData("bits per pixel", Vuforia.getBitsPerPixel(PIXEL_FORMAT.RGB565));
        telemetry.update();

        waitForStart();

//        relicTrackables.activate();

        while (opModeIsActive()) {
            while (vuforia.getFrameQueue().size() > 0) {
                VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
                Image capture = frame.getImage(0);
                frame.close();

                ByteBuffer bb = capture.getPixels();

                Bitmap bitmap = Bitmap.createBitmap(capture.getWidth(), capture.getHeight(), Bitmap.Config.RGB_565);
                bitmap.copyPixelsFromBuffer(bb);

                telemetry.update();
            }
        }

    }
}

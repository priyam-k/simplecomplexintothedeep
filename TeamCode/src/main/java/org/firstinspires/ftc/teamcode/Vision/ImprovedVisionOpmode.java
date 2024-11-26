package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;

@Config
@TeleOp(name="Improved Vision Opmode", group="Linear Opmode")
public class ImprovedVisionOpmode extends LinearOpMode {
    private PiplineForSample pipeline;
    private VisionPortal VP;

    private FtcDashboard dash;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the pipeline
        pipeline = new PiplineForSample();

        dash.getInstance().startCameraStream(pipeline,0);

        // Create the VisionPortal with the pipeline
        VP = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                pipeline
        );

        waitForStart();
        // Main loop during OpMode
        while (opModeIsActive()) {


        }





    }//run opmode end
}//class end

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@TeleOp(name="Vision OpMode", group="Linear Opmode")
public class VisionOpMode extends LinearOpMode {

    OpenCvWebcam webcam;
    SampleDetectionPipeline pipeline;

    @Override
    public void runOpMode() {
        // Get camera ID from the hardware map
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Initialize the pipeline
        pipeline = new SampleDetectionPipeline();

        // Set the pipeline to the webcam
        webcam.setPipeline(pipeline);

        // Open the camera device
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Start streaming the video feed at a resolution of 320x240
                webcam.startStreaming(320, 240);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", "Camera could not be opened!");
            }
        });

        // Wait for the start command
        waitForStart();

        while (opModeIsActive()) {
            // Get the detected stones from the pipeline
            ArrayList<SampleDetectionPipeline.AnalyzedStone> detectedStones = pipeline.getDetectedStones();

            // Display the detected stones and their properties on telemetry
            for (int i = 0; i < detectedStones.size(); i++) {
                SampleDetectionPipeline.AnalyzedStone stone = detectedStones.get(i);
                telemetry.addData("Stone " + i, "Color: %s, Angle: %.2f", stone.color, stone.angle);
            }

            // Update the telemetry
            telemetry.update();

            // Sleep briefly to avoid overloading the loop
            sleep(50);
        }

        // Stop the webcam when done
        webcam.stopStreaming();
    }
}

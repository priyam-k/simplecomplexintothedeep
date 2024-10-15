package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.SampleDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@TeleOp(name="Vision OpMode", group="Linear Opmode")
public class VisionOpMode extends LinearOpMode {

    OpenCvWebcam webcam;
    SampleDetectionPipeline pipeline;
    private Servo wristServo;
    public static double servoAngleOffset = 45; // offset starting position of servo to change servo deadzone
    public static String sampleColor = "yellow"; // color of the sample to detect

    public double degreesToTicks(double d){
        return d/270;
    }

    @Override
    public void runOpMode() {
        // initialize servo
        wristServo = hardwareMap.get(Servo.class, "Servo0");

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

        while (opModeInInit()) {
            // Get the detected stones from the pipeline
            ArrayList<SampleDetectionPipeline.AnalyzedStone> detectedStones = pipeline.getDetectedStones();
            double angle;
            // Display the detected stones and their properties on telemetry
            for (int i = 0; i < detectedStones.size(); i++) {
                SampleDetectionPipeline.AnalyzedStone stone = detectedStones.get(i);
                telemetry.addData("Stone " + i, "Color: %s, Angle: %.2f", stone.color, stone.angle);
            }

            SampleDetectionPipeline.AnalyzedStone targetSample;
            // get the first detected sample, only if there are detected samples
            if (!detectedStones.isEmpty()) {
                targetSample = detectedStones.get(0);
                // get the angle of the sample
                angle = targetSample.angle;
                // set the servo position to the angle of the sample, accounting for offset
                if (targetSample.color.equalsIgnoreCase(sampleColor))
                    wristServo.setPosition(degreesToTicks(270 - (angle)));
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

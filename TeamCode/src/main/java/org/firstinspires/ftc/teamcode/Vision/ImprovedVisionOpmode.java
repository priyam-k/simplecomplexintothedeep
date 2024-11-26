package org.firstinspires.ftc.teamcode.Vision;

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

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the pipeline
        pipeline = new PiplineForSample();

        // Create the VisionPortal with the pipeline
        VP = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                pipeline
        );


        // Main loop during OpMode
        while (opModeInInit()) {
            // Retrieve data from the pipeline
            PiplineForSample.YellowSampleData data = (PiplineForSample.YellowSampleData) pipeline.processFrame(null, 0);

            // Extract the center and angle
            Point center = data.center;
            double angle = data.angle;

            // Send telemetry data to the driver station
            telemetry.addData("Yellow Sample Center", "X: %.2f, Y: %.2f", center.x, center.y);
            telemetry.addData("Yellow Sample Angle", "%.2f degrees", angle);
            telemetry.update();
        }


        waitForStart();

        // Cleanup when OpMode stops
//        if (VP != null) {
//            VP.close();
//        }
    }//run opmode end
}//class end

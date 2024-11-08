package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystem.EnableHand;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.stream.Collectors;

@Config
@TeleOp(name="Vision OpMode", group="Linear Opmode")
public class VisionOpMode extends LinearOpMode {

    OpenCvWebcam webcam;
    SampleDetectionPipeline pipeline;

    Drivetrain drivetrain;
    EnableHand intake;

    public static double sampleTranslateGain,sampleStrafeGain;
    private Servo wristServo, armServo;
    public static double servoAngleOffset = 45; // offset starting position of servo to change servo deadzone
    public static String sampleColor = "yellow"; // color of the sample to detect

    public static double StrafeLine = 320;  //640

    public static double VerticalLine = 240; //480
    

    public double degreesToTicks(double d){
        return d/270;
    }

    public double dist(double x1, double y1, double x2, double y2){
        return Math.sqrt(Math.pow(x2-x1,2)+Math.pow(y2-y1,2));
    }

    private SampleDetectionPipeline.AnalyzedStone closestStone(ArrayList<SampleDetectionPipeline.AnalyzedStone> stones, double x, double y){
        if (stones.isEmpty()) return null;
        SampleDetectionPipeline.AnalyzedStone closest = stones.get(0);
        double leastDist = dist(x,y,closest.x,closest.y);
        double currDist;
        for (SampleDetectionPipeline.AnalyzedStone stone : stones){
            currDist = dist(x,y,stone.x,stone.y);
            if (currDist < leastDist){
                closest = stone;
                leastDist = currDist;
            }
        }
        return closest;
    }

    @Override
    public void runOpMode() {
        // initialize servo
        intake = new EnableHand();
        drivetrain = new Drivetrain();
        intake.init(hardwareMap);
        drivetrain.init(hardwareMap);
        intake.setSwingArmAngle(60);
        wristServo = hardwareMap.get(Servo.class, "Servo6");
//        armServo = hardwareMap.get(Servo.class, "Servo10");


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
                webcam.startStreaming(176,144);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", "Camera could not be opened!");
            }
        });

        // INIT LOOP STARTS HERE

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
            detectedStones = detectedStones.stream()
                    .filter(stone -> stone.color.equalsIgnoreCase(sampleColor))
                    .collect(Collectors.toCollection(ArrayList::new));

            targetSample = closestStone(detectedStones, StrafeLine, VerticalLine);

            if (targetSample != null) {
                angle = targetSample.angle;
                wristServo.setPosition(degreesToTicks(270 - (angle)));
                drivetrain.SampleAlign(targetSample.x, targetSample.y);
            } else {
                drivetrain.Brake();
            }

            // get the first detected sample, only if there are detected samples
//            if (!detectedStones.isEmpty()) {
//                targetSample = detectedStones.get(0);
//                // get the angle of the sample
//                angle = targetSample.angle;
//                // set the servo position to the angle of the sample, accounting for offset
//                if (targetSample.color.equalsIgnoreCase(sampleColor))
//                    wristServo.setPosition(degreesToTicks(270 - (angle)));
//            }




            //TODO: MAIN PEICE FO CODE OVER HERE

            //drivetrain.SampleAlign(pipeline.getCenter());










            // Update the telemetry
            telemetry.update();

            // Sleep briefly to avoid overloading the loop
            sleep(50);
        }

        // Stop the webcam when done
        webcam.stopStreaming();
    }
}

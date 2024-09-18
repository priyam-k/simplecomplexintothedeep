package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

@TeleOp(name="Testing Vision OpMode")
public class TestingVisionOpmode extends LinearOpMode{

    public VisionPortal portal;
    public VisionProcessor SampleDetectionPipeline;
    @Override
    public void runOpMode() throws InterruptedException {
        portal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), SampleDetectionPipeline);
        waitForStart();


    }
}

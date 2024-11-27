package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystem.EnableHand;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@TeleOp(name="Improved Vision Opmode for tunning", group="Linear Opmode")
public class ImprovedVisionOpmode extends LinearOpMode {
    private PiplineForSampleTuning pipeline;
    private VisionPortal VP;

    private FtcDashboard dash;

    public static int hueMin = 0,satMin = 100 ,valMin = 0,hueMax = 30,satMax = 255,valMax = 255;

    public static boolean Masked = true;

    private EnableHand hand;

    public static double Angle = 60;

    @Override
    public void runOpMode() throws InterruptedException {

        hand = new EnableHand();
        hand.init(hardwareMap);



        // Initialize the pipeline
        pipeline = new PiplineForSampleTuning();

        dash.getInstance().startCameraStream(pipeline,0);

        // Create the VisionPortal with the pipeline
        VP = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), pipeline);

        waitForStart();
        // Main loop during OpMode
        while (opModeIsActive()) {
            hand.setSwingArmAngle(Angle);

        }





    }//run opmode end
}//class end

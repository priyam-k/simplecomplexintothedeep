package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystem.EnableHand;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;

@Config
@TeleOp(name="AlignmentToSample", group="Linear Opmode")
public class AlignmentToSample extends LinearOpMode {
    private PiplineForAlignment pipeline;
    private VisionPortal VP;

    private FtcDashboard dash;

    public static double KpVertical = 0.0022,KpStraffe = -0.003;

    public Point PickupPixels;


    public static boolean Masked = true;
    public static boolean PidRunning = false;

    private EnableHand hand;
    private Drivetrain drive;

    private MultipleTelemetry tele;

    public static double Angle = 60;

    @Override
    public void runOpMode() throws InterruptedException {
        // Target positions of camera pixels
        PickupPixels= new Point(212.5,330.0);
        //hand intiallization
        hand = new EnableHand();
        drive = new Drivetrain();

        hand.init(hardwareMap);
        drive.init(hardwareMap);

        hand.scan4();




        // Initialize the pipeline
        pipeline = new PiplineForAlignment();
        dash = FtcDashboard.getInstance();

        dash.getInstance().startCameraStream(pipeline,0);


        // Create the VisionPortal with the pipeline
        VP = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), pipeline);

        tele  = new MultipleTelemetry(telemetry,dash.getTelemetry());

        waitForStart();
        // Main loop during OpMode
        while (opModeIsActive()) {
            hand.setSwingArmAngle(Angle);

            tele.addData("x Pos", pipeline.Center.x);
            tele.addData("Y Pos", pipeline.Center.y);


            if (PidRunning){
               double VerticalError =  PickupPixels.y - pipeline.Center.y;
               double StraffeError = PickupPixels.x - pipeline.Center.x;

              tele.addData("Vertical Error Pixels",VerticalError);
              tele.addData("Straffe Error Pixels",StraffeError);
              tele.addData("Vertical Motor Power",KpVertical*VerticalError);
              tele.addData("Straffe Motor Power", KpStraffe*StraffeError);

             drive.SampleAlign(KpVertical*VerticalError,KpStraffe*StraffeError);


            }

            tele.update();
        }//opmode loop end





    }//run opmode end
}//class end

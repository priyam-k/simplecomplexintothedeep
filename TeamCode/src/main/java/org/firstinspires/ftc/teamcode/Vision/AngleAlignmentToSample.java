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
@TeleOp(name="Angle AutoAlign", group="Linear Opmode")
public class AngleAlignmentToSample extends LinearOpMode {
    private PiplineForAlignment pipeline;
    private VisionPortal VP;

    private FtcDashboard dash;

    public static double KpVertical = 0.0022,KpStraffe = -0.0023;
    //cannot close small errors but does not overshoot

    public Point PickupPixels;

    public static int state = 0;

    private EnableHand hand;
    private Drivetrain drive;

    private MultipleTelemetry tele;

    public static double Angle = 60;

    @Override
    public void runOpMode() throws InterruptedException {
        // Target positions of camera pixels
        PickupPixels= new Point(240,340.0);
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


            tele.addData("x Pos", pipeline.Center.x);
            tele.addData("Y Pos", pipeline.Center.y);
            double VerticalError =  PickupPixels.y - pipeline.Center.y;
            double StraffeError = PickupPixels.x - pipeline.Center.x;
            tele.addData("Vertical Error Pixels",VerticalError);
            tele.addData("Straffe Error Pixels",StraffeError);
            tele.addData("Vertical Motor Power",KpVertical*VerticalError);
            tele.addData("Straffe Motor Power", KpStraffe*StraffeError);

            tele.addLine("0: Maksed, 1: pid running, 2: Arm turret increment running (dont change vars manually)");

            switch (state){
                case 0:
                    hand.setSwingArmAngle(Angle);
                    AlignmentToSample.Masked = true;
                    drive.Brake();
                    break;
                case 1:
                    hand.setSwingArmAngle(Angle);
                    AlignmentToSample.Masked = false;
                    drive.SampleAlign(KpVertical*VerticalError,KpStraffe*StraffeError);
                    break;
                case 2:
                    hand.setSwingAngleOnlyAngle(Angle);
                    AlignmentToSample.Masked = false;
                    drive.Brake();
                    hand.IntakeTurretAngleAutoAlign(StraffeError,20);
                    //20 pixel bound
                    break;

            }








            tele.update();
        }//opmode loop end





    }//run opmode end
}//class end

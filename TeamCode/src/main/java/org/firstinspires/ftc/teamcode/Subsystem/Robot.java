package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.AlignmentToSample;
import org.firstinspires.ftc.teamcode.Vision.PiplineForAlignment;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;

public class Robot implements Subsystem {
    public Drivetrain drive;
    public EnableHand hand;
    public MiggyUnLimbetedOuttake out;

    private VisionPortal VP;

    private PiplineForAlignment pipeline;

    private double timeforAutoAlignSeconds = 5.0;

    public double KpVertical = 0.002,KpStraffe = -0.0021;
    public Point PickupPixels;
    List<Subsystem> massInit;

    ElapsedTime timer;

    public Robot() {
        massInit = new ArrayList<>();
    }

    @Override
    public void init(HardwareMap hardwareMap) {
        drive = new Drivetrain();
        hand = new EnableHand();
        out = new MiggyUnLimbetedOuttake();


        massInit.add(drive);
        massInit.add(hand);
        massInit.add(out);

        massInit.add(drive);
        for (Subsystem s : massInit) {
            s.init(hardwareMap);
        }

        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        PickupPixels= new Point(212.5,340.0);

        pipeline = new PiplineForAlignment();

        VP = new VisionPortal.Builder().setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).addProcessor(pipeline).build();

//        VP.stopStreaming();//saving resources

    }

    public void initAutoAlign(){
        hand.setSwingArmAngle(60);
        hand.open();
        hand.scan4();// horizontal claw
        AlignmentToSample.Masked = false;
        AlignmentToSample.PidRunning = true;
    }

    public void AutoAlign(){
        VP.resumeStreaming(); //start stream

        timer.reset();
        timer.startTime();

        while(timer.seconds() < timeforAutoAlignSeconds){
            double VerticalError =  PickupPixels.y - pipeline.Center.y;
           double StraffeError = PickupPixels.x - pipeline.Center.x;

            drive.SampleAlign(KpVertical*VerticalError,KpStraffe*StraffeError);

        }// timer loop end



    }


    public void pickUp() {
        timer.reset();
        timer.startTime();
        while (timer.seconds() < 10) {

            if (timer.seconds() < 1) {
                hand.setHandTurretDegrees(90);
                hand.setHandTurretDegrees(5);
                hand.close();
            } else if (timer.seconds() < 3) {
                hand.setHandTurretDegrees(5);

            } else if (timer.seconds() < 4) {
                hand.open();
            } else if (timer.seconds() < 5) {
                hand.setHandTurretDegrees(60);
            }
        }
    }

    @Override
    public void update() {

    }

    @Override
    public void addTelemetry(Telemetry t) {

    }

}

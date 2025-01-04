package org.firstinspires.ftc.teamcode.Subsystem;

import static org.firstinspires.ftc.teamcode.Vision.AngleAlignmentToSample.ServoIncrement;

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
    public static int state = 0;
    public static double Angle = 60;
    public Drivetrain drive;
    public EnableHand hand;
    public MiggyUnLimbetedOuttake out;
    public double KpVertical = 0.0022, KpStraffe = -0.0003;
    public double FeedForward = 0.16;
    public Point PickupPixels;
    List<Subsystem> massInit;
    ElapsedTime timer;
    private VisionPortal VP;
    private PiplineForAlignment pipeline;
    private double timeforAutoAlignSeconds = 5.0;

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

        PickupPixels = new Point(212, 360.0);

        pipeline = new PiplineForAlignment();



        VP = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), pipeline);


        //  VP = new VisionPortal.Builder().setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).addProcessor(pipeline).build();

//        VP.stopStreaming();//saving resources

    }

    public void initAutoAlign() {
        hand.setSwingArmAngle(60);
        hand.open();
        hand.scan4();// horizontal claw
        AlignmentToSample.Masked = false;
        AlignmentToSample.PidRunning = true;
    }

    public void AutoAlign() {

        VP.resumeStreaming(); //start stream

        timer.reset();
        timer.startTime();
        this.initAutoAlign();

        while (timer.seconds() < timeforAutoAlignSeconds) {
            double VerticalError = PickupPixels.y - pipeline.Center.y;
            double StraffeError = PickupPixels.x - pipeline.Center.x;
            switch (state) {
                case 1:
                    hand.setSwingArmAngleAuton(Angle);
                    AlignmentToSample.Masked = false;
                    drive.SampleAlign(KpVertical * VerticalError, KpStraffe * StraffeError);
                    if (timer.seconds() > 2) {
                        state = 2;
                    }
                     break;
                case 2:
                    hand.setSwingAngleOnlyAngle(Angle);
                    AlignmentToSample.Masked = false;
                    drive.Brake();
                    //Arm turret ticks adjusting and getting what angle it is at
                    double angleArmTurr = hand.IntakeTurretAngleAutoAlign(StraffeError, 10, ServoIncrement);
                    //Send the reverse to the Claw turret
                    double ClawTargetAngle = 90 - angleArmTurr;

                    hand.ClawTurr.setPosition(hand.setHandTurretDegrees(ClawTargetAngle));
                    //20 pixel bound
                    //arm turret tick __ arm turret angle --> claw turret angle --> claw turret tick
                    if (timer.seconds() > 5){state = 3;}
                        break;
                case 3:
                    pickUpAuto();
                    break;

            }
        }// timer loop end
    }

    public void StopStreaming() {
        VP.stopStreaming();
    }


    public void pickUp() {
        //StopStreaming();
        drive.Brake();
        timer.reset();
        timer.startTime();
        while (timer.seconds() < 10) {

            if (timer.seconds() < 1) {
                hand.close();
                hand.setSwingAngleOnlyAngle(20);
            } else if (timer.seconds() < 3) {

                hand.setSwingAngleOnlyAngle(-5);
            } else if (timer.seconds() < 4) {
                hand.open();
            } else if (timer.seconds() < 5) {
                hand.setSwingAngleOnlyAngle(60);

            }
        }
    }
    public void pickUpAuto() {
        //StopStreaming();
        drive.Brake();
        timer.reset();
        timer.startTime();
        while (timer.seconds() < 10) {

            if (timer.seconds() < 1) {
                hand.close();
                hand.setSwingAngleOnlyAngle(20);
            } else if (timer.seconds() < 3) {

                hand.setSwingAngleOnlyAngle(-5);
            } else if (timer.seconds() < 4) {
                hand.open();
            } else if (timer.seconds() < 5) {
                hand.setSwingAngleOnlyAngle(60);

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

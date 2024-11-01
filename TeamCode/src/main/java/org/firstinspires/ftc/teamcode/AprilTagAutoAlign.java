package org.firstinspires.ftc.teamcode;

import android.transition.Slide;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystem.EnableHand;
import org.firstinspires.ftc.teamcode.Subsystem.MiggyUnLimbetedOuttake;

@Config
@Autonomous(name = "April Tag Auto Align")
public class AprilTagAutoAlign extends LinearOpMode {

    public static double RandomdistanceUnits = 27.0;

    public static double SlideTicks = 700;

    Drivetrain drive = new Drivetrain();
    EnableHand intake = new EnableHand();

    ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    MiggyUnLimbetedOuttake outake = new MiggyUnLimbetedOuttake();

    @Override
    public void runOpMode() throws InterruptedException {
       // MecanumDrive drive2 = new MecanumDrive(hardwareMap, new Pose2d(0, -31, Math.toRadians(270)));
        intake.init(hardwareMap);
        drive.init(hardwareMap);
        outake.init(hardwareMap);
        outake.autonInit();
        intake.setSwingArmAngleAuton(130);
//        Action myTrajectory = drive2.actionBuilder(drive2.pose)
//                .splineToLinearHeading(new Pose2d(-37, -33, Math.toRadians(90)), Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(-26, -9, Math.toRadians(90)), Math.toRadians(0))
//                .build();

        waitForStart();


        intake.setSwingArmAngleAuton(90);

        //moving back to see april tags
        time.reset();
        time.startTime();

        while(opModeIsActive()) {

            if (time.seconds()<1){
                drive.drive(-0.8);
                outake.PIDLoop(SlideTicks);
                outake.backAuton();
            }
            if (time.seconds()<3){
                drive.alignAprilTag(RandomdistanceUnits);
                outake.PIDLoop(SlideTicks);
            }
            else if (time.seconds()<3.5){
                outake.PIDLoop(SlideTicks-500);
            }
            else if (time.seconds()<3.6){
                outake.score();
            }
            else if(time.seconds()<4){
                outake.PIDLoop(0);
                //Actions.runBlocking(new SequentialAction(myTrajectory));
            }
            else if(time.seconds()<4.1){
                outake.SlidesBrake();
                outake.loiter1();
                outake.loiter2();
                outake.loiter3();
            }



        }
    }
}

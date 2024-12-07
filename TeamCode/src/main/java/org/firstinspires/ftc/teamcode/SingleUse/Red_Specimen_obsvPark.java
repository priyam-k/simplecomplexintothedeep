package org.firstinspires.ftc.teamcode.SingleUse;
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


public class Red_Specimen_obsvPark extends LinearOpMode {
    MecanumDrive drive3 = new MecanumDrive(hardwareMap, new Pose2d(0, -36, Math.toRadians(90)));
    Action myTrajectory = drive3.actionBuilder(drive3.pose)
            .splineToLinearHeading(new Pose2d(56, -60, Math.toRadians(180)), Math.toRadians(0))
            .build();
    public static double RandomdistanceUnits = 26.0;

    public static double SlideTicks = 640;

    Drivetrain drive2 = new Drivetrain();
    EnableHand hand = new EnableHand();
    ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    MiggyUnLimbetedOuttake out = new MiggyUnLimbetedOuttake();

    @Override
    public void runOpMode() throws InterruptedException {
        hand.init(hardwareMap);
        drive2.init(hardwareMap);
        drive2.initVisionPortal(hardwareMap);
        out.init(hardwareMap);
        out.autonInit();
        hand.setSwingArmAngleAuton(130);
        hand.open();
        while (!isStopRequested() && !opModeIsActive()) {
        }

        waitForStart();

        if (isStopRequested()) return;

//        hand.setSwingArmAngleAuton(85);

        //moving back to see april tags
        time.reset();
        time.startTime();

        while (opModeIsActive()) {

            // Movement of subsystems
            if (time.seconds() < 1.2) {
                drive2.drive(-0.5);
                out.PIDLoop(SlideTicks);
                out.backAuton();
                hand.setSwingArmAngleAuton(90);
            }
            //Going back to the correct location
            else if (time.seconds() < 3) {
                drive2.alignAprilTag(RandomdistanceUnits);
                out.PIDLoop(SlideTicks);
                //bringing down slides
            } else if (time.seconds() < 3.8) {
                out.PIDLoop(SlideTicks - 320);
            } else if (time.seconds() < 4.2) {
                out.score();
            } else if (time.seconds() < 5.5) {
                out.PIDLoop(0);
                out.loiter1();
                out.loiter2();
                out.loiter3();
                drive2.Brake();
                drive2.RESET(); //Get ready for adi runner
            }

            //poverty starts here
            else if (time.seconds() < 7.5) {
                out.SlidesBrake();
                out.autonInit();
                hand.setSwingArmAngleAuton(135);
            }
            else {

                Actions.runBlocking(new SequentialAction(myTrajectory));
            }
        }
    }
}


package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Config
@Autonomous (name = "MeepMeep")
public class TrajectoryTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(48, 48, Math.toRadians(0)));

        Action myTrajectory = drive.actionBuilder(drive.pose)
                .turn(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(58, 55, Math.toRadians(90)), Math.toRadians(0))
//                .lineToY(45)
//                .splineToLinearHeading(new Pose2d(58, 55, Math.toRadians(225)), Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(58, 45, Math.toRadians(-90)), Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(58, 55, Math.toRadians(225)), Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(56, 45, Math.toRadians(300)), Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(58, 55, Math.toRadians(225)), Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(23, 20, 0), Math.toRadians(30))

//                .turn(Math.toRadians(-70))
//                .waitSeconds(0.7)
//                .lineToX(-48) //-58
//                .turn(Math.toRadians(80))
//                .waitSeconds(1)
//                .turn(Math.toRadians(-185))
//                .waitSeconds(1)
//                .turn(Math.toRadians(-150))
//                .waitSeconds(1)
//                .turn(Math.toRadians(150))
//                .waitSeconds(1)
//                .turn(Math.toRadians(155))
//                .waitSeconds(1)
//                .turn(Math.toRadians(-155))
//                .turn(Math.toRadians(-155))
//                .lineToY(-5)
                .build();
        waitForStart();

        if(isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(myTrajectory));
    }
    //                        .lineToY(24)
//                        .waitSeconds(2.0)
//                        .lineToY(40)
//                        .turn(Math.toRadians(-90))
//                        .lineToX(-48)
//                        .turn(Math.toRadians(90))
//                        .lineToY(33)
//                        .waitSeconds(1.0)
//                        .lineToY(40)
//                        .turn(Math.toRadians(90))
//                        .lineToX(55)
//                        .turn(Math.toRadians(-90))
//                        .lineToY(63)
//                        .waitSeconds(1.0)
//                        .lineToY(40)
//                        .turn(Math.toRadians(-90))
//                        .lineToX(-58)
//                        .turn(Math.toRadians(90))
//                        .lineToY(33)
//                        .waitSeconds(1.0)
//                        .lineToY(40)
//                        .turn(Math.toRadians(90))
//                        .lineToX(55)
//                        .turn(Math.toRadians(-90))
//                        .lineToY(63)
//                        .waitSeconds(1.0)
//                        .lineToY(40)
//                        .turn(Math.toRadians(-90))
//                        .lineToX(-68)
//                        .turn(Math.toRadians(90))
//                        .lineToY(33)
//                        .waitSeconds(1.0)
//                        .lineToY(40)
//                        .turn(Math.toRadians(90))
//                        .lineToX(55)
//                        .turn(Math.toRadians(-90))
//                        .lineToY(63)
    }


package org.firstinspires.ftc.teamcode.RoadRunner.autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.SlideControl;
import org.firstinspires.ftc.teamcode.Subsystem.EnableHand;
import org.firstinspires.ftc.teamcode.Subsystem.MiggyUnLimbetedOuttake;
import org.firstinspires.ftc.teamcode.Subsystem.StateMachines;

@Autonomous(name = "BlueAlliance1_2")
public class BlueAllaince1_1 extends LinearOpMode {


    EnableHand hand;
    MiggyUnLimbetedOuttake out;

    @Override
    public void runOpMode() throws InterruptedException {
        SlideControl slides = new SlideControl();
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 40, Math.toRadians(270)));
        hand = new EnableHand();


        hand = new EnableHand();
        out = new MiggyUnLimbetedOuttake();

        out.init(hardwareMap);
        hand.init(hardwareMap);

        Action myTrajectory = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(51.1, 38.5, Math.toRadians(-90)), Math.toRadians(-90))
//                .waitSeconds(1)
//                .
//
//                .lineToX(23)
                .build();
        Action Traj2 = drive.actionBuilder(drive.pose)
               .splineToLinearHeading(new Pose2d(53.2, 47.9, Math.toRadians(225)), Math.toRadians(100))
                .build();
        Action parkTraj = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(53.2, 47.9, Math.toRadians(225)), Math.toRadians(100))
                .turn(Math.toRadians(15))
                .build();


        while (!isStopRequested() && !opModeIsActive()) {
        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        myTrajectory,


                        telemetryPacket -> {
                            telemetry.addLine("Scan 1");
                            telemetry.update();
                            hand.scan1();
                            return false;
                        },
                        new SleepAction(0.8),

                        telemetryPacket -> {
                            telemetry.addLine("Scan 2");
                            telemetry.update();
                            hand.scan2();
                            return false;
                        },
                        new SleepAction(0.8),

                        telemetryPacket -> {
                            telemetry.addLine("Scan 3");
                            telemetry.update();
                            hand.scan3();
                            return false;
                        },
                        new SleepAction(0.8),

                        telemetryPacket -> {
                            telemetry.addLine("Scan 4");
                            telemetry.update();
                            hand.scan4();
                            return false;
                        },
                        new SleepAction(0.8),

                        telemetryPacket -> {
                            telemetry.addLine("Hovering");
                            telemetry.update();
                            hand.hover();
                            return false;
                        },
                        new SleepAction(0.8),

                        telemetryPacket -> {
                            telemetry.addLine("Picking up 1");
                            telemetry.update();
                            hand.pickup1();
                            return false;
                        },
                        new SleepAction(0.8),

                        telemetryPacket -> {
                            telemetry.addLine("Picking up 2");
                            telemetry.update();
                            hand.pickup2();
                            return false;
                        },
                        new SleepAction(0.8),

                        telemetryPacket -> {
                            telemetry.addLine("Transferring 1");
                            telemetry.update();
                            hand.transfer1();
                            return false;
                        },
                        new SleepAction(0.8),

                        telemetryPacket -> {
                            telemetry.addLine("Transferring 1.5");
                            telemetry.update();
                            hand.transfer1point5();
                            return false;
                        },
                        new SleepAction(0.8),

                        telemetryPacket -> {
                            telemetry.addLine("Transferring 2");
                            telemetry.update();
                            hand.transfer2();
                            return false;
                        }, new SleepAction(0.8),

                        telemetryPacket -> {
                            telemetry.addLine("Loiter");
                            telemetry.update();
                            hand.loiter();
                            return false;
                        }, new SleepAction(0.8),

                        new SequentialAction(
                        telemetryPacket -> {
                            telemetry.addLine("Loiter 1");
                            telemetry.update();
                            out.loiter1();
                            return false;
                        }, new SleepAction(0.8),

                        telemetryPacket -> {
                            telemetry.addLine("Loiter 2");
                            telemetry.update();
                            out.loiter2();
                            return false;
                        }, new SleepAction(0.8),
                        telemetryPacket -> {
                            telemetry.addLine("Loiter 3");
                            telemetry.update();
                            out.loiter3();
                            return false;
                        }, new SleepAction(0.8),
                        telemetryPacket -> {
                            telemetry.addLine("Transfer 1");
                            telemetry.update();
                            out.transfer1();
                            return false;
                        },new SleepAction(0.8),
                        telemetryPacket -> {
                            telemetry.addLine("Transfer 2");
                            telemetry.update();
                            out.transfer2();
                            return false;
                        }, new SleepAction(0.8),
                        //Goes to basket pistion
                        Traj2,



                        telemetryPacket -> {
                            telemetry.addLine("Back 1");
                            telemetry.update();
                            out.back1();
                            return false;
                            }, new SleepAction(0.8),
                        telemetryPacket -> {
                            telemetry.addLine("Back 2");
                            telemetry.update();
                            out.back2();
                            return false;
                            }, new SleepAction(0.8),
                        telemetryPacket -> {
                            telemetry.addLine("Score");
                            telemetry.update();
                            out.score();
                            return false;
                            }, new SleepAction(0.8)

                )
        ));
//       Actions.runBlocking(slides.slideUp());


    }
}

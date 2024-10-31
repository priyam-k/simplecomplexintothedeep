package org.firstinspires.ftc.teamcode.RoadRunner.autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystem.EnableHand;
import org.firstinspires.ftc.teamcode.Subsystem.MiggyUnLimbetedOuttake;

@Autonomous(name = "BlueAlliance1_2")
public class BlueAllaince1_1 extends LinearOpMode {


    EnableHand hand;
    MiggyUnLimbetedOuttake out;
    DcMotorEx slideMotorRight, slideMotorLeft;
    public double kP = 0.006;
    public double targetPos = 4400;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 40, Math.toRadians(0)));
        hand = new EnableHand();

        hand = new EnableHand();
        out = new MiggyUnLimbetedOuttake();


        out.init(hardwareMap);
        hand.init(hardwareMap);

        Action myTrajectory = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(50.8, 41.5, Math.toRadians(-90)), Math.toRadians(-90))
//                .waitSeconds(1)
//                .
//
//                .lineToX(23)
                .build();
        MecanumDrive drive2 = new MecanumDrive(hardwareMap, new Pose2d(50.8, 41.5, Math.toRadians(-90)));
        Action Traj2 = drive2.actionBuilder(drive2.pose)
                .splineToLinearHeading(new Pose2d(59.2 , 56.2, Math.toRadians(230)), Math.toRadians(100)) //56.5, 54
                .build();
//        Action parkTraj = drive.actionBuilder(drive.pose)
//                .splineToLinearHeading(new Pose2d(50.6, 34.3, Math.toRadians(225)), Math.toRadians(100))
//                .turn(Math.toRadians(15))
//                .lineToX(23)
//                .build();


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
                            hand.hoverAuto();
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
                        },
                        new SleepAction(0.8),
                        new SequentialAction(
                                telemetryPacket -> {
                                    telemetry.addLine("Loiter 1");
                                    telemetry.update();
                                    out.loiter1();
                                    return false;
                                },
                                new SleepAction(0.8),

                                telemetryPacket -> {
                                    telemetry.addLine("Loiter 2");
                                    telemetry.update();
                                    out.loiter2();
                                    return false;
                                },
                                new SleepAction(0.8),
                                telemetryPacket -> {
                                    telemetry.addLine("Loiter 3");
                                    telemetry.update();
                                    out.loiter3();
                                    return false;
                                },
                                new SleepAction(0.8),
                                telemetryPacket -> {
                                    telemetry.addLine("Transfer 1");
                                    telemetry.update();
                                    out.transfer1();
                                    return false;
                                },
                                new SleepAction(0.8),
                                telemetryPacket -> {
                                    telemetry.addLine("Transfer 2");
                                    telemetry.update();
                                    out.transfer2();
                                    return false;
                                },
                                new SleepAction(0.8),
                                telemetryPacket -> {
                                    telemetry.addLine("Loiter");
                                    telemetry.update();
                                    hand.loiter();
                                    return false;
                                },
                                new SleepAction(0.3),
                                //Goes to basket pistion
                                Traj2,
                                new SleepAction(0.8),
                                telemetryPacket -> {
                                    telemetry.addLine("Back 1");
                                    telemetry.update();
                                    out.back1();
                                    return false;
                                },
                                new SleepAction(0.8),
                                telemetryPacket -> {
                                    telemetry.addData("Slides current pos ", out.currentPos);
                                    telemetry.update();

                                        out.PIDLoopAuto(3000);

                                    return false;
                                },
                                new SleepAction(0.8),
                                telemetryPacket -> {
                                    telemetry.addLine("Back 2");
                                    telemetry.update();
                                    out.back2();
                                    return false;
                                },

                                new SleepAction(0.8),
                                telemetryPacket -> {
                                    telemetry.addLine("Score");
                                    telemetry.update();
                                    out.score();
                                    return false;
                                },
                                telemetryPacket -> {
                                    telemetry.addLine("Set slides power to 0");
                                    telemetry.update();
                                    out.SlidesBrake();
                                    return false;
                                },
                                new SleepAction(0.8),
                                telemetryPacket -> {
                                    telemetry.addLine("back 1 after placing");
                                    telemetry.update();
                                    out.back1();
                                    return false;
                                },
                                telemetryPacket -> {
                                    telemetry.addLine("Slides down");
                                    telemetry.update();
                                    out.PIDLoopAuto(0);
                                    return false;
                                }


                        )
                ));
//       Actions.runBlocking(slides.slideUp());


    }
}

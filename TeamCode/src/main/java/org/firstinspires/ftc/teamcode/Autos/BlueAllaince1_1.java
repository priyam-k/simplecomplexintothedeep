package org.firstinspires.ftc.teamcode.Autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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


public class BlueAllaince1_1 extends LinearOpMode {


    EnableHand hand;
    MiggyUnLimbetedOuttake out;
    DcMotorEx slideMotorRight, slideMotorLeft;
    public double kP = 0.006;
    public double targetPos = 1600;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 34, Math.toRadians(90)));
        hand = new EnableHand();
        out = new MiggyUnLimbetedOuttake();
        out.init(hardwareMap);
        hand.init(hardwareMap);

        Action myTrajectory = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(0, 45, Math.toRadians(-5)), Math.toRadians(0))
                .build();
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 45, Math.toRadians(-5)));
        Action Traj2 = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(50.5, 41, Math.toRadians(-95)), Math.toRadians(-90)) //56.5, 54
                .build();
        drive = new MecanumDrive(hardwareMap, new Pose2d(50.5, 41, Math.toRadians(-95)));
        Action Traj3 = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(60.3, 49, Math.toRadians(225)), Math.toRadians(100)) //56.5, 54
                .build();
        drive = new MecanumDrive(hardwareMap, new Pose2d(60.3, 49, Math.toRadians(225)));
        Action Traj4 = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(62, 39.7, Math.toRadians(-90)), Math.toRadians(-90))
                .build();


        while (!isStopRequested() && !opModeIsActive()) {
        }
        out.autonInit();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        myTrajectory,
                        Traj2,
                        telemetryPacket -> {
                            telemetry.addLine("Scan 1");
                            telemetry.update();
                            hand.scan1();
                            return false;
                        },
                        new SleepAction(0.7),

                        telemetryPacket -> {
                            telemetry.addLine("Scan 2");
                            telemetry.update();
                            hand.scan2();
                            return false;
                        },
                        new SleepAction(0.7),

                        telemetryPacket -> {
                            telemetry.addLine("Scan 3");
                            telemetry.update();
                            hand.scan3();
                            return false;
                        },
                        new SleepAction(0.7),

                        telemetryPacket -> {
                            telemetry.addLine("Scan 4");
                            telemetry.update();
                            hand.scan4();
                            return false;
                        },
                        new SleepAction(0.7),

                        telemetryPacket -> {
                            telemetry.addLine("Hovering");
                            telemetry.update();
                            hand.hoverAuto();
                            return false;
                        },
                        new SleepAction(0.7),
                        telemetryPacket -> {
                            telemetry.addLine("Picking up 1");
                            telemetry.update();
                            hand.pickup1();
                            return false;
                        },
                        new SleepAction(0.7),

                        telemetryPacket -> {
                            telemetry.addLine("Picking up 2");
                            telemetry.update();
                            hand.pickup2();
                            return false;

                        },
                        new SleepAction(0.7),

                        telemetryPacket -> {
                            telemetry.addLine("Transferring 1");
                            telemetry.update();
                            hand.transfer1();
                            return false;
                        },
                        new SleepAction(0.7),
                        telemetryPacket -> {
                            telemetry.addLine(" Set Turret position too center");
                            telemetry.update();
                            hand.turrSet0Auton();
                            return false;
                        },
                        new SleepAction(0.7),
                        telemetryPacket -> {
                            telemetry.addLine("Transferring 1.5");
                            telemetry.update();
                            hand.transfer1point5();
                            return false;
                        },
                        new SleepAction(0.7),

                        telemetryPacket -> {
                            telemetry.addLine("Transferring 2");
                            telemetry.update();
                            hand.transfer2();
                            return false;
                        },
                        new SleepAction(0.7),
                        new SequentialAction(
                                telemetryPacket -> {
                                    telemetry.addLine("Loiter 1");
                                    telemetry.update();
                                    out.loiter1();
                                    return false;
                                },
                                new SleepAction(0.7),

                                telemetryPacket -> {
                                    telemetry.addLine("Loiter 2");
                                    telemetry.update();
                                    out.loiter2();
                                    return false;
                                },
                                new SleepAction(0.7),
                                telemetryPacket -> {
                                    telemetry.addLine("Loiter 3");
                                    telemetry.update();
                                    out.loiter3();
                                    return false;
                                },
                                new SleepAction(0.7),
                                telemetryPacket -> {
                                    telemetry.addLine("Transfer 1");
                                    telemetry.update();
                                    out.transfer1();
                                    return false;
                                },
                                new SleepAction(0.7),
                                telemetryPacket -> {
                                    telemetry.addLine("Transfer 2");
                                    telemetry.update();
                                    out.transfer2();
                                    return false;
                                },
                                new SleepAction(0.7),
                                telemetryPacket -> {
                                    telemetry.addLine("Loiter");
                                    telemetry.update();
                                    hand.loiter();
                                    return false;
                                },
                                new SleepAction(0.3),
                                //Goes to basket pistion
                                Traj3,
                                new SleepAction(0.7),
                                telemetryPacket -> {
                                    telemetry.addLine("Back 1");
                                    telemetry.update();
                                    out.back1();
                                    return false;
                                },
                                new SleepAction(0.7),
                                telemetryPacket -> {
                                    out.PIDLoopAuto(1000);
                                    telemetry.addData("Slides current pos ", out.currentPos);
                                    telemetry.update();
                                    return false;
                                },
                                new SleepAction(0.7),
                                telemetryPacket -> {
                                    telemetry.addLine("Back 2");
                                    telemetry.update();
                                    out.back2();
                                    return false;
                                },

                                new SleepAction(0.7),
                                telemetryPacket -> {
                                    telemetry.addLine("Score");
                                    telemetry.update();
                                    out.score();
                                    return false;
                                },
                                new SleepAction(0.7),
                                telemetryPacket -> {
                                    telemetry.addLine("Set slides power to 0");
                                    telemetry.update();
                                    out.PIDLoopAuto(0);
                                    out.SlidesBrake();
                                    return false;
                                },
                                new SleepAction(0.7),
                                telemetryPacket -> {
                                    telemetry.addLine("back 1 after placing");
                                    telemetry.update();
                                    out.back1();
                                    return false;
                                },
                                new SleepAction(0.7),
                                telemetryPacket -> {
                                    telemetry.addLine("Slides down");
                                    telemetry.update();
                                    out.PIDLoopAuto(0);
                                    return false;
                                }
                        )));
    }
}

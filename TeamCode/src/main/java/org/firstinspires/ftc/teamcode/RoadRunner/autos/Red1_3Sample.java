package org.firstinspires.ftc.teamcode.RoadRunner.autos;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystem.EnableHand;
import org.firstinspires.ftc.teamcode.Subsystem.MiggyUnLimbetedOuttake;
@Autonomous (name = "A Red Sample 1+3")

public class Red1_3Sample extends LinearOpMode{
    Drivetrain drive = new Drivetrain();
    EnableHand hand = new EnableHand();
    MiggyUnLimbetedOuttake out = new MiggyUnLimbetedOuttake();


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive3 = new MecanumDrive(hardwareMap, new Pose2d(-30.75, -63.25, Math.toRadians(90)));
//
//        hand.init(hardwareMap);
//        drive.init(hardwareMap);
//        drive.initVisionPortal(hardwareMap);
//        out.init(hardwareMap);
//        out.transfer2(); // CLOSES THE CLAW
//        hand.setSwingArmAngleAuton(130);
//        hand.open();

        Action basketTraj = drive3.actionBuilder(drive3.pose)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-56, -52, Math.toRadians(40)), Math.toRadians(280))
                .build();
        drive3 = new MecanumDrive(hardwareMap, new Pose2d(-56, -52, Math.toRadians(40)));

        Action sample1Traj = drive3.actionBuilder(drive3.pose)
                .splineToLinearHeading(new Pose2d(-51.8, -43.5, Math.toRadians(85)), Math.toRadians(90))
                .build();
        drive3 = new MecanumDrive(hardwareMap, new Pose2d(-51.8, -43.5, Math.toRadians(85)));

        Action sample2Traj = drive3.actionBuilder(drive3.pose)
                .splineToLinearHeading(new Pose2d(-64, -43, Math.toRadians(85)), Math.toRadians(90))
                .build();
        drive3 = new MecanumDrive(hardwareMap, new Pose2d(-64, -43, Math.toRadians(85)));

        Action sample3Traj = drive3.actionBuilder(drive3.pose)
                .splineToLinearHeading(new Pose2d(-47, -25.5, Math.toRadians(180)), Math.toRadians(90))
                .build();
        drive3 = new MecanumDrive(hardwareMap, new Pose2d(-47, -25.5, Math.toRadians(180)));
        while (!isStopRequested() && !opModeIsActive()) {
        }

        waitForStart();

//        out.autonInit();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        basketTraj,
                      /*  new SleepAction(0.3),
                        telemetryPacket -> {
                            telemetry.addLine("Back 1");
                            telemetry.update();
                            out.back1();
                            return false;
                        },
                        telemetryPacket -> {
                            telemetry.addLine("Back 2");
                            telemetry.update();
                            out.back2Auton();
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
                            out.backAuton();
                            return false;
                        },

                        new SleepAction(0.7),
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
                        new SleepAction(0.3),
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
                            out.PIDLoop(-20);
                            return false;
                        }
                        ,
                        new SleepAction(0.7)
                        ,
                        telemetryPacket -> {
                            telemetry.addLine("Slides brake");
                            telemetry.update();
                            out.SlidesBrake();
                            return false;
                        },*/

                        // PLACED PRELOADED SAMPLE


                        //START PATHING TO SAMPLE 1

                        sample1Traj,

//                        telemetryPacket -> {
//                            telemetry.addLine("Scan 1");
//                            telemetry.update();
//                            hand.scan1();
//                            return false;
//                        },
//                        new SleepAction(0.2),
//                        telemetryPacket -> {
//                            telemetry.addLine("Scan 2");
//                            telemetry.update();
//                            hand.scan2();
//                            return false;
//                        },
//                        new SleepAction(0.2),
//                        telemetryPacket -> {
//                            telemetry.addLine("Scan 3");
//                            telemetry.update();
//                            //hand.scan3();
//                            hand.close();
//                            return false;
//                        },
//                        new SleepAction(0.3),
//                        telemetryPacket -> {
//                            telemetry.addLine("Hovering");
//                            telemetry.update();
//                            hand.hoverAuto();
//                            return false;
//                        },
//                        new SleepAction(0.4),
//                        telemetryPacket -> {
//                            telemetry.addLine("Picking up 1");
//                            telemetry.update();
//                            hand.autonPickup1();
//                            return false;
//                        },
//                        new SleepAction(0.5),
//                        telemetryPacket -> {
//                            telemetry.addLine("Picking up 1");
//                            telemetry.update();
//                            hand.pickup2Auton();
//                            return false;
//                        },
//                        new SleepAction(0.3),
//
//                        telemetryPacket -> {
//                            telemetry.addLine("Picking up 2");
//                            telemetry.update();
//                            hand.open();
//                            return false;
//
//                        },
//                        new SleepAction(0.7),
//                        telemetryPacket -> {
//                            telemetry.addLine("Transferring 1");
//                            telemetry.update();
//                            hand.transfer1();
//                            return false;
//                        },
//                        telemetryPacket -> {
//                            telemetry.addLine(" Set Turret position to center");
//                            telemetry.update();
//                            hand.turrSet0Auton();
//                            return false;
//                        },
//                        telemetryPacket -> {
//                            telemetry.addLine("Transferring 1.5");
//                            telemetry.update();
//                            hand.transfer1point5();
//                            return false;
//                        },
//
//                        telemetryPacket -> {
//                            telemetry.addLine("Transferring 2");
//                            telemetry.update();
//                            hand.autonTransfer2();
//                            return false;
//                        },
//                        new SleepAction(0.7),
//                        telemetryPacket -> {
//                            telemetry.addLine("Loiter 1");
//                            telemetry.update();
//                            out.loiter1();
//                            return false;
//                        },
//                        new SleepAction(0.3),
//
//                        telemetryPacket -> {
//                            telemetry.addLine("Loiter 2");
//                            telemetry.update();
//                            out.loiter2();
//                            return false;
//                        },
//                        new SleepAction(0.3),
//                        telemetryPacket -> {
//                            telemetry.addLine("Loiter 3");
//                            telemetry.update();
//                            out.loiter3();
//                            return false;
//                        },
//                        new SleepAction(0.7),
//                        telemetryPacket -> {
//                            telemetry.addLine("Transfer 1");
//                            telemetry.update();
//                            out.transfer1();
//                            return false;
//                        },
//                        new SleepAction(0.3),
//                        telemetryPacket -> {
//                            telemetry.addLine("Transfer 2");
//                            telemetry.update();
//                            out.transfer2();
//                            return false;
//                        },
//                        new SleepAction(0.3),
//                        telemetryPacket -> {
//                            telemetry.addLine("Hand released sample");
//                            telemetry.update();
//                            hand.close();
//                            return false;
//                        },
//                        // SPLINE TO BASKET
                        basketTraj,
                        // DEPOSIT IN BASKET
//                        new SleepAction(0.3),
//                        telemetryPacket -> {
//                            telemetry.addLine("Back 1");
//                            telemetry.update();
//                            out.back1();
//                            return false;
//                        },
//                        telemetryPacket -> {
//                            telemetry.addLine("Back 2");
//                            telemetry.update();
//                            out.back2Auton();
//                            return false;
//                        },
//                        new SleepAction(0.7),
//                        telemetryPacket -> {
//                            out.PIDLoopAuto(1000);
//                            telemetry.addData("Slides current pos ", out.currentPos);
//                            telemetry.update();
//                            return false;
//                        },
//                        new SleepAction(0.7),
//                        telemetryPacket -> {
//                            telemetry.addLine("Back 2");
//                            telemetry.update();
//                            out.backAuton();
//                            return false;
//                        },
//
//                        new SleepAction(0.7),
//                        telemetryPacket -> {
//                            telemetry.addLine("Score");
//                            telemetry.update();
//                            out.score();
//                            return false;
//                        },
//                        telemetryPacket -> {
//                            telemetry.addLine("Set slides power to 0");
//                            telemetry.update();
//                            out.SlidesBrake();
//                            return false;
//                        },
//                        new SleepAction(0.3),
//                        telemetryPacket -> {
//                            telemetry.addLine("back 1 after placing");
//                            telemetry.update();
//                            out.back1();
//                            return false;
//                        },
//                        new SleepAction(0.7),
//                        telemetryPacket -> {
//                            telemetry.addLine("Slides down");
//                            telemetry.update();
//                            out.PIDLoop(-20);
//                            return false;
//                        }
//                        ,
//                        new SleepAction(0.7)
//                        ,
//                        telemetryPacket -> {
//                            telemetry.addLine("Slides brake");
//                            telemetry.update();
//                            out.SlidesBrake();
//                            return false;
//                        },


                        // PLACED SAMPLE 1


                        //START PATHING TO SAMPLE 2

                        sample2Traj,
//
//                        telemetryPacket -> {
//                            telemetry.addLine("Scan 1");
//                            telemetry.update();
//                            hand.scan1();
//                            return false;
//                        },
//                        new SleepAction(0.2),
//                        telemetryPacket -> {
//                            telemetry.addLine("Scan 2");
//                            telemetry.update();
//                            hand.scan2();
//                            return false;
//                        },
//                        new SleepAction(0.2),
//                        telemetryPacket -> {
//                            telemetry.addLine("Scan 3");
//                            telemetry.update();
//                            //hand.scan3();
//                            hand.close();
//                            return false;
//                        },
//                        new SleepAction(0.3),
//                        telemetryPacket -> {
//                            telemetry.addLine("Hovering");
//                            telemetry.update();
//                            hand.hoverAuto();
//                            return false;
//                        },
//                        new SleepAction(0.4),
//                        telemetryPacket -> {
//                            telemetry.addLine("Picking up 1");
//                            telemetry.update();
//                            hand.autonPickup1();
//                            return false;
//                        },
//                        new SleepAction(0.5),
//                        telemetryPacket -> {
//                            telemetry.addLine("Picking up 1");
//                            telemetry.update();
//                            hand.pickup2Auton();
//                            return false;
//                        },
//                        new SleepAction(0.3),
//
//                        telemetryPacket -> {
//                            telemetry.addLine("Picking up 2");
//                            telemetry.update();
//                            hand.open();
//                            return false;
//
//                        },
//                        new SleepAction(0.7),
//                        telemetryPacket -> {
//                            telemetry.addLine("Transferring 1");
//                            telemetry.update();
//                            hand.transfer1();
//                            return false;
//                        },
//                        telemetryPacket -> {
//                            telemetry.addLine(" Set Turret position to center");
//                            telemetry.update();
//                            hand.turrSet0Auton();
//                            return false;
//                        },
//                        telemetryPacket -> {
//                            telemetry.addLine("Transferring 1.5");
//                            telemetry.update();
//                            hand.transfer1point5();
//                            return false;
//                        },
//
//                        telemetryPacket -> {
//                            telemetry.addLine("Transferring 2");
//                            telemetry.update();
//                            hand.autonTransfer2();
//                            return false;
//                        },
//                        new SleepAction(0.7),
//                        telemetryPacket -> {
//                            telemetry.addLine("Loiter 1");
//                            telemetry.update();
//                            out.loiter1();
//                            return false;
//                        },
//                        new SleepAction(0.3),
//
//                        telemetryPacket -> {
//                            telemetry.addLine("Loiter 2");
//                            telemetry.update();
//                            out.loiter2();
//                            return false;
//                        },
//                        new SleepAction(0.3),
//                        telemetryPacket -> {
//                            telemetry.addLine("Loiter 3");
//                            telemetry.update();
//                            out.loiter3();
//                            return false;
//                        },
//                        new SleepAction(0.7),
//                        telemetryPacket -> {
//                            telemetry.addLine("Transfer 1");
//                            telemetry.update();
//                            out.transfer1();
//                            return false;
//                        },
//                        new SleepAction(0.3),
//                        telemetryPacket -> {
//                            telemetry.addLine("Transfer 2");
//                            telemetry.update();
//                            out.transfer2();
//                            return false;
//                        },
//                        new SleepAction(0.3),
//                        telemetryPacket -> {
//                            telemetry.addLine("Hand released sample");
//                            telemetry.update();
//                            hand.close();
//                            return false;
//                        },
//                        // SPLINE TO BASKET
                        basketTraj,
//                        // DEPOSIT SAMPLE 2 IN BASKET
//                        new SleepAction(0.3),
//                        telemetryPacket -> {
//                            telemetry.addLine("Back 1");
//                            telemetry.update();
//                            out.back1();
//                            return false;
//                        },
//                        telemetryPacket -> {
//                            telemetry.addLine("Back 2");
//                            telemetry.update();
//                            out.back2Auton();
//                            return false;
//                        },
//                        new SleepAction(0.7),
//                        telemetryPacket -> {
//                            out.PIDLoopAuto(1000);
//                            telemetry.addData("Slides current pos ", out.currentPos);
//                            telemetry.update();
//                            return false;
//                        },
//                        new SleepAction(0.7),
//                        telemetryPacket -> {
//                            telemetry.addLine("Back 2");
//                            telemetry.update();
//                            out.backAuton();
//                            return false;
//                        },
//
//                        new SleepAction(0.7),
//                        telemetryPacket -> {
//                            telemetry.addLine("Score");
//                            telemetry.update();
//                            out.score();
//                            return false;
//                        },
//                        telemetryPacket -> {
//                            telemetry.addLine("Set slides power to 0");
//                            telemetry.update();
//                            out.SlidesBrake();
//                            return false;
//                        },
//                        new SleepAction(0.3),
//                        telemetryPacket -> {
//                            telemetry.addLine("back 1 after placing");
//                            telemetry.update();
//                            out.back1();
//                            return false;
//                        },
//                        new SleepAction(0.7),
//                        telemetryPacket -> {
//                            telemetry.addLine("Slides down");
//                            telemetry.update();
//                            out.PIDLoop(-20);
//                            return false;
//                        }
//                        ,
//                        new SleepAction(0.7)
//                        ,
//                        telemetryPacket -> {
//                            telemetry.addLine("Slides brake");
//                            telemetry.update();
//                            out.SlidesBrake();
//                            return false;
//                        },
//
//
//                        // PLACED SAMPLE 2 IN THE BASKET
//
//
//                        //START PATHING TO SAMPLE 3

                        sample3Traj,

//                        telemetryPacket -> {
//                            telemetry.addLine("Scan 1");
//                            telemetry.update();
//                            hand.scan1();
//                            return false;
//                        },
//                        new SleepAction(0.2),
//                        telemetryPacket -> {
//                            telemetry.addLine("Scan 2");
//                            telemetry.update();
//                            hand.scan2();
//                            return false;
//                        },
//                        new SleepAction(0.2),
//                        telemetryPacket -> {
//                            telemetry.addLine("Scan 3");
//                            telemetry.update();
//                            //hand.scan3();
//                            hand.close();
//                            return false;
//                        },
//                        new SleepAction(0.3),
//                        telemetryPacket -> {
//                            telemetry.addLine("Hovering");
//                            telemetry.update();
//                            hand.hoverAuto();
//                            return false;
//                        },
//                        new SleepAction(0.4),
//                        telemetryPacket -> {
//                            telemetry.addLine("Picking up 1");
//                            telemetry.update();
//                            hand.autonPickup1();
//                            return false;
//                        },
//                        new SleepAction(0.5),
//                        telemetryPacket -> {
//                            telemetry.addLine("Picking up 1");
//                            telemetry.update();
//                            hand.pickup2Auton();
//                            return false;
//                        },
//                        new SleepAction(0.3),
//
//                        telemetryPacket -> {
//                            telemetry.addLine("Picking up 2");
//                            telemetry.update();
//                            hand.open();
//                            return false;
//
//                        },
//                        new SleepAction(0.7),
//                        telemetryPacket -> {
//                            telemetry.addLine("Transferring 1");
//                            telemetry.update();
//                            hand.transfer1();
//                            return false;
//                        },
//                        telemetryPacket -> {
//                            telemetry.addLine(" Set Turret position to center");
//                            telemetry.update();
//                            hand.turrSet0Auton();
//                            return false;
//                        },
//                        telemetryPacket -> {
//                            telemetry.addLine("Transferring 1.5");
//                            telemetry.update();
//                            hand.transfer1point5();
//                            return false;
//                        },
//
//                        telemetryPacket -> {
//                            telemetry.addLine("Transferring 2");
//                            telemetry.update();
//                            hand.autonTransfer2();
//                            return false;
//                        },
//                        new SleepAction(0.7),
//                        telemetryPacket -> {
//                            telemetry.addLine("Loiter 1");
//                            telemetry.update();
//                            out.loiter1();
//                            return false;
//                        },
//                        new SleepAction(0.3),
//
//                        telemetryPacket -> {
//                            telemetry.addLine("Loiter 2");
//                            telemetry.update();
//                            out.loiter2();
//                            return false;
//                        },
//                        new SleepAction(0.3),
//                        telemetryPacket -> {
//                            telemetry.addLine("Loiter 3");
//                            telemetry.update();
//                            out.loiter3();
//                            return false;
//                        },
//                        new SleepAction(0.7),
//                        telemetryPacket -> {
//                            telemetry.addLine("Transfer 1");
//                            telemetry.update();
//                            out.transfer1();
//                            return false;
//                        },
//                        new SleepAction(0.3),
//                        telemetryPacket -> {
//                            telemetry.addLine("Transfer 2");
//                            telemetry.update();
//                            out.transfer2();
//                            return false;
//                        },
//                        new SleepAction(0.3),
//                        telemetryPacket -> {
//                            telemetry.addLine("Hand released sample");
//                            telemetry.update();
//                            hand.close();
//                            return false;
//                        },
//                        // SPLINE TO BASKET
                        basketTraj
//                        // DEPOSIT IN BASKET
//                        new SleepAction(0.3),
//                        telemetryPacket -> {
//                            telemetry.addLine("Back 1");
//                            telemetry.update();
//                            out.back1();
//                            return false;
//                        },
//                        telemetryPacket -> {
//                            telemetry.addLine("Back 2");
//                            telemetry.update();
//                            out.back2Auton();
//                            return false;
//                        },
//                        new SleepAction(0.7),
//                        telemetryPacket -> {
//                            out.PIDLoopAuto(1000);
//                            telemetry.addData("Slides current pos ", out.currentPos);
//                            telemetry.update();
//                            return false;
//                        },
//                        new SleepAction(0.7),
//                        telemetryPacket -> {
//                            telemetry.addLine("Back 2");
//                            telemetry.update();
//                            out.backAuton();
//                            return false;
//                        },
//
//                        new SleepAction(0.7),
//                        telemetryPacket -> {
//                            telemetry.addLine("Score");
//                            telemetry.update();
//                            out.score();
//                            return false;
//                        },
//                        telemetryPacket -> {
//                            telemetry.addLine("Set slides power to 0");
//                            telemetry.update();
//                            out.SlidesBrake();
//                            return false;
//                        },
//                        new SleepAction(0.3),
//                        telemetryPacket -> {
//                            telemetry.addLine("back 1 after placing");
//                            telemetry.update();
//                            out.back1();
//                            return false;
//                        },
//                        new SleepAction(0.7),
//                        telemetryPacket -> {
//                            telemetry.addLine("Slides down");
//                            telemetry.update();
//                            out.PIDLoop(-20);
//                            return false;
//                        }
//                        ,
//                        new SleepAction(0.7)
//                        ,
//                        telemetryPacket -> {
//                            telemetry.addLine("Slides brake");
//                            telemetry.update();
//                            out.SlidesBrake();
//                            return false;
//                        }
                        // PLACED ALL SAMPLES IN BASKET
                )
        );
    }
}

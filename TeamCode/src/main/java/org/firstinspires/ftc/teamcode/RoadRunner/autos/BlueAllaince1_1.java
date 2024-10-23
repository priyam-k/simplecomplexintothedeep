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

@Autonomous(name = "BlueAllaince1_2")
public class BlueAllaince1_1 extends LinearOpMode {


    EnableHand hand;
    MiggyUnLimbetedOuttake out;

    @Override
    public void runOpMode() throws InterruptedException {
        SlideControl slides = new SlideControl();
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 40, Math.toRadians(0)));
        hand = new EnableHand();


        hand = new EnableHand();
        out = new MiggyUnLimbetedOuttake();

        out.init(hardwareMap);
        hand.init(hardwareMap);

        Action myTrajectory = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(51.1, 38.4, Math.toRadians(-90)), Math.toRadians(-90))
//                .waitSeconds(1)
//                .splineToLinearHeading(new Pose2d(53.2, 47.9, Math.toRadians(225)), Math.toRadians(100))
//                .waitSeconds(1)
//                .turn(Math.toRadians(15))
//
//                .lineToX(23)
                .build();


        while (!isStopRequested() && !opModeIsActive()) {
        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        myTrajectory,
                        telemetryPacket -> {
                            hand.loiter();
                            hand.scan1();
                            return false;
                        },
                        telemetryPacket -> {
                            hand.scan2();
                            return false;
                        },
                        telemetryPacket -> {
                            hand.scan3();
                            return false;
                        },
                        telemetryPacket -> {
                            hand.scan4();
                            return false;
                        },
                        telemetryPacket -> {
                            hand.hover();
                            return false;
                        },
                        telemetryPacket -> {
                            hand.pickup1();
                            return false;
                        },
                        telemetryPacket -> {
                            hand.pickup2();
                            return false;
                        },
                        telemetryPacket -> {
                            hand.transfer1();
                            return false;
                        },
                        telemetryPacket -> {
                            hand.transfer2();
                            return false;
                        })
//                        new SequentialAction(
//                        telemetryPacket -> {
//                            out.loiter1();
//                            return true;
//                        },
//                        telemetryPacket -> {
//                            out.loiter2();
//                            return true;
//                        },
//                        telemetryPacket -> {
//                            out.loiter3();
//                            return true;
//                        },
//                        telemetryPacket -> {
//                            out.transfer1();
//                            return true;
//                        },
//                        telemetryPacket -> {
//                            out.transfer2();
//                            return true;
//                        },
//                        telemetryPacket -> {
//                            out.back1();
//                            return true;
//                        },
//                        telemetryPacket -> {
//                            out.back2();
//                            return true;
//                        }
//                        telemetryPacket -> {
//                            out.score();
//                            return true;
//                        }


//                )
        );
//        Actions.runBlocking(slides.slideUp());


    }
}

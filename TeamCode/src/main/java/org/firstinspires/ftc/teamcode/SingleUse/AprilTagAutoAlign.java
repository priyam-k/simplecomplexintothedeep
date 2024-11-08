package org.firstinspires.ftc.teamcode.SingleUse;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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

    public static double RandomdistanceUnits = 26.5;

    public static double SlideTicks = 640;

    Drivetrain drive = new Drivetrain();
    EnableHand hand = new EnableHand();
    ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    MiggyUnLimbetedOuttake out = new MiggyUnLimbetedOuttake();

    @Override
    public void runOpMode() throws InterruptedException {
        //MecanumDrive drive2 = new MecanumDrive(hardwareMap, new Pose2d(0, 34, Math.toRadians(90)));

        hand.init(hardwareMap);
        drive.init(hardwareMap);
        drive.initVisionPortal(hardwareMap);
        out.init(hardwareMap);
        out.autonInit();
        hand.setSwingArmAngleAuton(130);
//
//        Action myTrajectory = drive2.actionBuilder(drive2.pose).splineToLinearHeading(new Pose2d(0, 45, Math.toRadians(0)), Math.toRadians(0)).build();
//
//        drive2 = new MecanumDrive(hardwareMap, new Pose2d(0, 45, Math.toRadians(0)));
//        Action Traj2 = drive2.actionBuilder(drive2.pose).splineToLinearHeading(new Pose2d(50.6, 39.7, Math.toRadians(-90)), Math.toRadians(-90)) //56.5, 54
//                .build();
//        drive2 = new MecanumDrive(hardwareMap, new Pose2d(50.6, 39.7, Math.toRadians(-90)));
//        Action Traj3 = drive2.actionBuilder(drive2.pose).splineToLinearHeading(new Pose2d(59.8, 57.5, Math.toRadians(230)), Math.toRadians(100)) //56.5, 54
//                .build();
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
                drive.drive(-0.5);
                out.PIDLoop(SlideTicks);
                out.backAuton();
                hand.setSwingArmAngleAuton(90);
            }
            //Going back to the correct location
            else if (time.seconds() < 3) {
                drive.alignAprilTag(RandomdistanceUnits);
                out.PIDLoop(SlideTicks);
                //bringing down slides
            } else if (time.seconds() < 3.8) {
                out.PIDLoop(SlideTicks - 320);
            }
            else if (time.seconds() < 4.2) {
                out.score();
            }
           else if (time.seconds() < 5.5) {
                out.PIDLoop(0);
                out.loiter1();
                out.loiter2();
                out.loiter3();
                drive.Brake();
                drive.RESET(); //Get ready for adi runner
            }
           //poverty starts here
            else if(time.seconds()<7.5) {
                out.SlidesBrake();
                drive.toPoint(7000,0,-90);
            }
            else if (time.seconds()<7.6){
                drive.Brake();
                drive.RESET();
            }
            else if(time.seconds() < 9 ){
                drive.toPoint(33000,-5000,0);
            }
            else if(time.seconds() < 10.5){
                drive.toPoint(33000,-5000,-135);
            }
            //should end here
            else if(time.seconds() < 10.6){
                drive.Brake();
                drive.RESET();
            }
            else if(time.seconds() < 12.0){
                drive.toPoint(-7000,-2000,0);
                hand.setSwingArmAngleAdiRunner(15,0.6);
            }
            else if(time.seconds() < 14.0){
                drive.toPoint(-11000,8000,0);
                hand.setSwingArmAngleAdiRunner(180,0.43);
            }
            // 1. Hand turret viertical

            //First block drive.toPoint(-7000,-2000,0);

            //Scoring position drive.toPoint(-11000,8000,0);






























//            else {
//                out.autonInit();
//                hand.setSwingArmAngleAuton(135);
////                Actions.runBlocking(new SequentialAction(myTrajectory, Traj2,
////
////                        telemetryPacket -> {
////                            telemetry.addLine("Scan 1");
////                            telemetry.update();
////                            hand.scan1();
////                            return false;
////                        }, new SleepAction(0.8),
////
////                        telemetryPacket -> {
////                            telemetry.addLine("Scan 2");
////                            telemetry.update();
////                            hand.scan2();
////                            return false;
////                        }, new SleepAction(0.8),
////
////                        telemetryPacket -> {
////                            telemetry.addLine("Scan 3");
////                            telemetry.update();
////                            hand.scan3();
////                            return false;
////                        }, new SleepAction(0.8),
////
////                        telemetryPacket -> {
////                            telemetry.addLine("Scan 4");
////                            telemetry.update();
////                            hand.scan4();
////                            return false;
////                        }, new SleepAction(0.8),
////
////                        telemetryPacket -> {
////                            telemetry.addLine("Hovering");
////                            telemetry.update();
////                            hand.hoverAuto();
////                            return false;
////                        }, new SleepAction(0.8),
////
////                        telemetryPacket -> {
////                            telemetry.addLine("Picking up 1");
////                            telemetry.update();
////                            hand.pickup1();
////                            return false;
////                        }, new SleepAction(0.8),
////
////                        telemetryPacket -> {
////                            telemetry.addLine("Picking up 2");
////                            telemetry.update();
////                            hand.pickup2();
////                            return false;
////
////                        }, new SleepAction(0.8),
////
////                        telemetryPacket -> {
////                            telemetry.addLine("Transferring 1");
////                            telemetry.update();
////                            hand.transfer1();
////                            return false;
////                        }, new SleepAction(0.8),
////
////                        telemetryPacket -> {
////                            telemetry.addLine("Transferring 1.5");
////                            telemetry.update();
////                            hand.transfer1point5();
////                            return false;
////                        }, new SleepAction(0.8),
////
////                        telemetryPacket -> {
////                            telemetry.addLine("Transferring 2");
////                            telemetry.update();
////                            hand.transfer2();
////                            return false;
////                        }, new SleepAction(0.8), new SequentialAction(telemetryPacket -> {
////                    telemetry.addLine("Loiter 1");
////                    telemetry.update();
////                    out.loiter1();
////                    return false;
////                }, new SleepAction(0.8),
////
////                        telemetryPacket -> {
////                            telemetry.addLine("Loiter 2");
////                            telemetry.update();
////                            out.loiter2();
////                            return false;
////                        }, new SleepAction(0.8), telemetryPacket -> {
////                    telemetry.addLine("Loiter 3");
////                    telemetry.update();
////                    out.loiter3();
////                    return false;
////                }, new SleepAction(0.8), telemetryPacket -> {
////                    telemetry.addLine("Transfer 1");
////                    telemetry.update();
////                    out.transfer1();
////                    return false;
////                }, new SleepAction(0.8), telemetryPacket -> {
////                    telemetry.addLine("Transfer 2");
////                    telemetry.update();
////                    out.transfer2();
////                    return false;
////                }, new SleepAction(0.8), telemetryPacket -> {
////                    telemetry.addLine("Loiter");
////                    telemetry.update();
////                    hand.loiter();
////                    return false;
////                }, new SleepAction(0.3),
////                        //Goes to basket pistion
////                        Traj3, new SleepAction(0.8), telemetryPacket -> {
////                    telemetry.addLine("Back 1");
////                    telemetry.update();
////                    out.back1();
////                    return false;
////                }, new SleepAction(0.8), telemetryPacket -> {
////                    telemetry.addData("Slides current pos ", out.currentPos);
////                    telemetry.update();
////
////                    out.PIDLoopAuto(3000);
////
////                    return false;
////                }, new SleepAction(0.8), telemetryPacket -> {
////                    telemetry.addLine("Back 2");
////                    telemetry.update();
////                    out.back2();
////                    return false;
////                },
////
////                        new SleepAction(0.8), telemetryPacket -> {
////                    telemetry.addLine("Score");
////                    telemetry.update();
////                    out.score();
////                    return false;
////                }, telemetryPacket -> {
////                    telemetry.addLine("Set slides power to 0");
////                    telemetry.update();
////                    out.SlidesBrake();
////                    return false;
////                }, new SleepAction(0.8), telemetryPacket -> {
////                    telemetry.addLine("back 1 after placing");
////                    telemetry.update();
////                    out.back1();
////                    return false;
////                }, telemetryPacket -> {
////                    telemetry.addLine("Slides down");
////                    telemetry.update();
////                    out.PIDLoopAuto(0);
////                    return false;
////                }
////                )));
//            }
        }
    }
}

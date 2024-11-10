package org.firstinspires.ftc.teamcode;

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
@Autonomous(name = "BLUE April Tag Auto Align")
public class AprilTagAutoAlign_Blue extends LinearOpMode {

    public static double RandomdistanceUnits = 26.0;

    public static double SlideTicks = 640;

    Drivetrain drive2 = new Drivetrain();
    EnableHand hand = new EnableHand();
    ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    MiggyUnLimbetedOuttake out = new MiggyUnLimbetedOuttake();

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive3 = new MecanumDrive(hardwareMap, new Pose2d(0, 36, Math.toRadians(90)));

        hand.init(hardwareMap);
        drive2.init(hardwareMap);
        drive2.initVisionPortal(hardwareMap);
        out.init(hardwareMap);
        out.autonInit();
        hand.setSwingArmAngleAuton(130);
        hand.open();

        Action myTrajectory = drive3.actionBuilder(drive3.pose)
                .splineToLinearHeading(new Pose2d(0, 45, Math.toRadians(-5)), Math.toRadians(0))
                .build();

        drive3 = new MecanumDrive(hardwareMap, new Pose2d(0, 45, Math.toRadians(-5)));
        Action sample1Traj = drive3.actionBuilder(drive3.pose)
                .splineToLinearHeading(new Pose2d(51.8, 43, Math.toRadians(-95)), Math.toRadians(-90)) //56.5, 54
                .build();
        drive3 = new MecanumDrive(hardwareMap, new Pose2d(51.8, 43, Math.toRadians(-95)));
        Action basketTraj = drive3.actionBuilder(drive3.pose)
                .splineToLinearHeading(new Pose2d(61, 47.5, Math.toRadians(220)), Math.toRadians(100)) //56.5, 54
                .build();
        drive3 = new MecanumDrive(hardwareMap, new Pose2d(61, 47.5, Math.toRadians(220)));
        Action sample2Traj = drive3.actionBuilder(drive3.pose)
                .splineToLinearHeading(new Pose2d(69, 43, Math.toRadians(-95)), Math.toRadians(-90))
                .build();
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
//                drive2.toPoint(7000, 0, -90);
//            } else if (time.seconds() < 7.6) {
//                drive2.Brake();
//                drive2.RESET();
//            }
////            else if (time.seconds() < 9) {
//                drive.toPoint(33000, -5000, 0);
//                drive.alignAprilTag(10);
//
//            } else if (time.seconds() < 12.5) {
//                drive.toPoint(33000, -5000, -90);
//
//            }
            //should end here
//            else if (time.seconds() < 13) {
//
//                drive2.Brake();
//                drive2.RESET();
//            }
            //poverty end here


//
//
//            else if (time.seconds() < 14.0) {
//                drive.toPoint(-9000, 0, 0);
//                hand.setSwingArmAngleAdiRunner(30, 0.43);
//            } else if (time.seconds() < 15.0) {
//                hand.setSwingArmAngleAdiRunner(-5, 0.43);
//            } else if (time.seconds() < 15.5) {
//                hand.close();
//            } else if (time.seconds() < 17.5) {
//                drive.toPoint(-8000, 0, -45);
//                hand.setSwingArmAngleAdiRunner(180, 0.43);
//            } else if (time.seconds() < 18) {
//                out.loiter1();
//                drive.toPoint(2000, -18000, -45);
//            } else if (time.seconds() < 18.01) {
//                drive.Brake();
//            } else if (time.seconds() < 18.25) {
//                out.loiter2();
//            } else if (time.seconds() < 18.5) {
//                out.loiter3();
//            } else if (time.seconds() < 19) {
//                out.transfer1();
//                hand.loiter();
//            } else if (time.seconds() < 19.75) {
//                out.backAuton();
//            } else if (time.seconds() < 20.75) {
//                out.PIDLoop(1400);
//            } else if (time.seconds() < 21.75) {
//                out.score();
//                drive.toPoint(-16000, 3000, -45);
//            } else if (time.seconds() < 21.76) {
//                drive.Brake();
//            }

            // 1. Hand turret vertical

            //First block drive.toPoint(-7000,-2000,0);

            //Scoring position drive.toPoint(-11000,8000,0);


            else {
//                out.autonInit();
                Actions.runBlocking(
                        new SequentialAction(
                                myTrajectory,
                                sample1Traj,
                                telemetryPacket -> {
                                    telemetry.addLine("Scan 1");
                                    telemetry.update();
                                    hand.scan1();
                                    return false;
                                },
                                new SleepAction(0.2),
                                telemetryPacket -> {
                                    telemetry.addLine("Scan 2");
                                    telemetry.update();
                                    hand.scan2();
                                    return false;
                                },
                                new SleepAction(0.2),
                                telemetryPacket -> {
                                    telemetry.addLine("Scan 3");
                                    telemetry.update();
                                    //hand.scan3();
                                    hand.close();
                                    return false;
                                },
                                new SleepAction(0.3),
                                telemetryPacket -> {
                                    telemetry.addLine("Hovering");
                                    telemetry.update();
                                    hand.hoverAuto();
                                    return false;
                                },
                                new SleepAction(0.4),
                                telemetryPacket -> {
                                    telemetry.addLine("Picking up 1");
                                    telemetry.update();
                                    hand.autonPickup1();
                                    return false;
                                },
                                new SleepAction(0.5),
                                telemetryPacket -> {
                                    telemetry.addLine("Picking up 1");
                                    telemetry.update();
                                    hand.pickup2Auton();
                                    return false;
                                },
                                new SleepAction(0.3),

                                telemetryPacket -> {
                                    telemetry.addLine("Picking up 2");
                                    telemetry.update();
                                    hand.open();
                                    return false;

                                },
                                new SleepAction(0.7),
                                telemetryPacket -> {
                                    telemetry.addLine("Transferring 1");
                                    telemetry.update();
                                    hand.transfer1();
                                    return false;
                                },
                                telemetryPacket -> {
                                    telemetry.addLine(" Set Turret position to center");
                                    telemetry.update();
                                    hand.turrSet0Auton();
                                    return false;
                                },
                                telemetryPacket -> {
                                    telemetry.addLine("Transferring 1.5");
                                    telemetry.update();
                                    hand.transfer1point5();
                                    return false;
                                },

                                telemetryPacket -> {
                                    telemetry.addLine("Transferring 2");
                                    telemetry.update();
                                    hand.autonTransfer2();
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
                                        new SleepAction(0.3),

                                        telemetryPacket -> {
                                            telemetry.addLine("Loiter 2");
                                            telemetry.update();
                                            out.loiter2();
                                            return false;
                                        },
                                        new SleepAction(0.3),
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
                                        new SleepAction(0.3),
                                        telemetryPacket -> {
                                            telemetry.addLine("Transfer 2");
                                            telemetry.update();
                                            out.transfer2();
                                            return false;
                                        },
                                        new SleepAction(0.3),
                                        telemetryPacket -> {
                                            telemetry.addLine("Loiter");
                                            telemetry.update();
                                            hand.close();
                                            return false;
                                        },
                                        //Goes to basket pistion
                                        basketTraj,
                                        new SleepAction(0.3),
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
                                        }


                                        //END OF SAMPLE 1 SCORING
                                        //START OF SAMPLE 2 PICKUP

//                                        ,
//                                        sample2Traj
//                                        ,
//                                        telemetryPacket -> {
//                                            telemetry.addLine("Scan 1");
//                                            telemetry.update();
//                                            hand.scan1();
//                                            return false;
//                                        },
//                                        new SleepAction(0.7),
//
//                                        telemetryPacket -> {
//                                            telemetry.addLine("Scan 2");
//                                            telemetry.update();
//                                            hand.scan2();
//                                            return false;
//                                        },
//                                        new SleepAction(0.7),
//                                        telemetryPacket -> {
//                                            telemetry.addLine("Scan 3");
//                                            telemetry.update();
//                                            //hand.scan3();
//                                            hand.close();
//                                            return false;
//                                        },
//                                        new SleepAction(0.7),
//
//                                        telemetryPacket -> {
//                                            telemetry.addLine("Hovering");
//                                            telemetry.update();
//                                            hand.hoverAuto();
//                                            return false;
//                                        },
//                                        new SleepAction(0.7),
//                                        telemetryPacket -> {
//                                            telemetry.addLine("Picking up 1");
//                                            telemetry.update();
//                                            hand.autonPickup1();
//                                            return false;
//                                        },
//                                        new SleepAction(0.7),
//
//                                        telemetryPacket -> {
//                                            telemetry.addLine("Picking up 1");
//                                            telemetry.update();
//                                            hand.pickup2Auton();
//                                            return false;
//                                        },
//                                        new SleepAction(0.7),
//
//                                        telemetryPacket -> {
//                                            telemetry.addLine("Picking up 2");
//                                            telemetry.update();
//                                            hand.open();
//                                            return false;
//
//                                        },
//                                        new SleepAction(0.7),
//
//                                        telemetryPacket -> {
//                                            telemetry.addLine("Transferring 1");
//                                            telemetry.update();
//                                            hand.transfer1();
//                                            return false;
//                                        },
//                                        new SleepAction(0.3),
//
//                                        telemetryPacket -> {
//                                            telemetry.addLine(" Set Turret position too center");
//                                            telemetry.update();
//                                            hand.turrSet0Auton();
//                                            return false;
//                                        },
//                                        new SequentialAction(
//                                                telemetryPacket -> {
//                                                    telemetry.addLine("Loiter 1");
//                                                    telemetry.update();
//                                                    out.loiter1();
//                                                    return false;
//                                                },
//                                                new SleepAction(0.7),
//
//                                                telemetryPacket -> {
//                                                    telemetry.addLine("Loiter 2");
//                                                    telemetry.update();
//                                                    out.loiter2();
//                                                    return false;
//                                                },
//                                                new SleepAction(0.7),
//                                                telemetryPacket -> {
//                                                    telemetry.addLine("Loiter 3");
//                                                    telemetry.update();
//                                                    out.loiter3();
//                                                    return false;
//                                                },
//                                        new SleepAction(0.7),
//                                        telemetryPacket -> {
//                                            telemetry.addLine("Transferring 1.5");
//                                            telemetry.update();
//                                            hand.transfer1point5();
//                                            return false;
//                                        },
//                                        new SleepAction(0.7),
//
//                                        telemetryPacket -> {
//                                            telemetry.addLine("Transferring 2");
//                                            telemetry.update();
//                                            hand.autonTransfer2();
//                                            return false;
//                                        },
//                                        new SleepAction(0.7),
//
//                                                new SleepAction(0.7),
//                                                telemetryPacket -> {
//                                                    telemetry.addLine("Transfer 1");
//                                                    telemetry.update();
//                                                    out.transfer1();
//                                                    return false;
//                                                },
//                                                new SleepAction(0.7),
//                                                telemetryPacket -> {
//                                                    telemetry.addLine("Transfer 2");
//                                                    telemetry.update();
//                                                    out.transfer2();
//                                                    return false;
//                                                },
//                                                new SleepAction(0.7),
//                                                telemetryPacket -> {
//                                                    telemetry.addLine("Loiter");
//                                                    telemetry.update();
//                                                    hand.close();
//                                                    return false;
//                                                },
//                                                new SleepAction(0.3),
//                                                //Goes to basket pistion
//                                                basketTraj,
//                                                new SleepAction(0.7),
//                                                telemetryPacket -> {
//                                                    telemetry.addLine("Back 1");
//                                                    telemetry.update();
//                                                    out.back1();
//                                                    return false;
//                                                },
//                                                telemetryPacket -> {
//                                                    telemetry.addLine("Back 2");
//                                                    telemetry.update();
//                                                    out.back2Auton();
//                                                    return false;
//                                                },
//                                                new SleepAction(0.7),
//                                                telemetryPacket -> {
//                                                    out.PIDLoopAuto(1000);
//                                                    telemetry.addData("Slides current pos ", out.currentPos);
//                                                    telemetry.update();
//                                                    return false;
//                                                },
//                                                new SleepAction(0.7),
//                                                telemetryPacket -> {
//                                                    telemetry.addLine("Back 2");
//                                                    telemetry.update();
//                                                    out.backAuton();
//                                                    return false;
//                                                },
//
//                                                new SleepAction(0.7),
//                                                telemetryPacket -> {
//                                                    telemetry.addLine("Score");
//                                                    telemetry.update();
//                                                    out.score();
//                                                    return false;
//                                                },
//                                                new SleepAction(0.7),
//                                                telemetryPacket -> {
//                                                    telemetry.addLine("Set slides power to 0");
//                                                    telemetry.update();
//                                                    out.SlidesBrake();
//                                                    return false;
//                                                },
//                                                new SleepAction(0.7),
//                                                telemetryPacket -> {
//                                                    telemetry.addLine("back 1 after placing");
//                                                    telemetry.update();
//                                                    out.back1();
//                                                    return false;
//                                                },
//                                                new SleepAction(0.7),
//                                                telemetryPacket -> {
//                                                    telemetry.addLine("Slides down");
//                                                    telemetry.update();
//                                                    out.PIDLoop(-20);
//                                                    return false;
//                                                }
//                                                ,
//                                                new SleepAction(2)
//                                                ,
//                                                telemetryPacket -> {
//                                                    telemetry.addLine("Slides brake");
//                                                    telemetry.update();
//                                                    out.SlidesBrake();
//                                                    return false;
//                                                }
                                )));
            }
        }
    }
}

package org.firstinspires.ftc.teamcode.RoadRunner.autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystem.EnableHand;
import org.firstinspires.ftc.teamcode.Subsystem.MiggyUnLimbetedOuttake;


public class REALSpecimenAutoRed extends LinearOpMode {
    Drivetrain drive = new Drivetrain();
    EnableHand hand = new EnableHand();
    MiggyUnLimbetedOuttake out = new MiggyUnLimbetedOuttake();


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive3 = new MecanumDrive(hardwareMap, new Pose2d(16.75, -60.75, Math.toRadians(270)));


        hand.init(hardwareMap);
        drive.init(hardwareMap);
        drive.initVisionPortal(hardwareMap);
        out.init(hardwareMap);
        out.transfer2(); // CLOSES THE CLAW
        hand.setSwingArmAngleAuton(130);
        hand.open();


        Pose2d highChamberPose = new Pose2d(0, -35, Math.toRadians(270));
        Pose2d obsZonePose = new Pose2d(47, -52, Math.toRadians(86));
        Pose2d parkPose = new Pose2d(47, -61, Math.toRadians(270));
        Pose2d obsZonePickupPose= new Pose2d(47, -59, Math.toRadians(90));

        Action highChamberTraj = drive3.actionBuilder(drive3.pose).splineToLinearHeading(highChamberPose, Math.toRadians(90)).build();
        drive3 = new MecanumDrive(hardwareMap, highChamberPose);

        Action obsZoneTraj = drive3.actionBuilder(drive3.pose).splineToLinearHeading(obsZonePose, Math.toRadians(270)).build();
        drive3 = new MecanumDrive(hardwareMap, obsZonePickupPose);

        Action highChamberTraj2 = drive3.actionBuilder(drive3.pose).splineToLinearHeading(highChamberPose, Math.toRadians(90)).build();
        drive3 = new MecanumDrive(hardwareMap, obsZonePose);

        Action obsZonePickupTraj= drive3.actionBuilder(drive3.pose).lineToY(-59).build();
        drive3 = new MecanumDrive(hardwareMap, highChamberPose);
        Action parkTraj = drive3.actionBuilder(drive3.pose).splineToLinearHeading(parkPose, Math.toRadians(270)).build();

        while (!isStopRequested() && !opModeIsActive()) {
        }
        waitForStart();

        out.autonInit();

        if (isStopRequested()) return;

//        Action basket1Action = new SequentialAction(highChamberTraj, new SleepAction(0.3), telemetryPacket -> {
//            telemetry.addLine("Back 1");
//            telemetry.update();
//            out.back1();
//            return false;
//        }, telemetryPacket -> {
//            telemetry.addLine("Back 2");
//            telemetry.update();
//            out.back2Auton();
//            return false;
//        }, new SleepAction(0.7), telemetryPacket -> {
////            out.PIDLoop(1000);
//            telemetry.addData("Slides current pos", out.currentPos);
//            telemetry.update();
//            return false;
//        }, new SleepAction(0.7), telemetryPacket -> {
//            telemetry.addLine("Back 2");
//            telemetry.update();
//            out.backAuton();
//            return false;
//        }, new SleepAction(0.7), telemetryPacket -> {
//            telemetry.addLine("Score");
//            telemetry.update();
//            out.score();
//            return false;
//        }, telemetryPacket -> {
//            telemetry.addLine("Set slides power to 0");
//            telemetry.update();
//            out.SlidesBrake();
//            return false;
//        }, new SleepAction(0.3), telemetryPacket -> {
//            telemetry.addLine("back 1 after placing");
//            telemetry.update();
//            out.back1();
//            return false;
//        }, new SleepAction(0.7), telemetryPacket -> {
//            telemetry.addLine("Slides down");
//            telemetry.update();
////            out.PIDLoop(0);
//            return false;
//        }, new SleepAction(0.7), telemetryPacket -> {
//            telemetry.addLine("Slides brake");
//            telemetry.update();
//            out.SlidesBrake();
//            return false;
//        });
//        Action sample1Action = new SequentialAction(obsZoneTraj, telemetryPacket -> {
//            telemetry.addLine("Scan 1");
//            telemetry.update();
//            hand.scan1();
//            return false;
//        }, new SleepAction(0.2), telemetryPacket -> {
//            telemetry.addLine("Scan 2");
//            telemetry.update();
//            hand.scan2();
//            return false;
//        }, new SleepAction(0.2), telemetryPacket -> {
//            telemetry.addLine("Scan 3");
//            telemetry.update();
//            //hand.scan3();
//            hand.close();
//            return false;
//        }, new SleepAction(0.3), telemetryPacket -> {
//            telemetry.addLine("Hovering");
//            telemetry.update();
//            hand.hoverAuto();
//            return false;
//        }, new SleepAction(0.4), telemetryPacket -> {
//            telemetry.addLine("Picking up 1");
//            telemetry.update();
//            hand.autonPickup1();
//            return false;
//        }, new SleepAction(0.5), telemetryPacket -> {
//            telemetry.addLine("Picking up 1");
//            telemetry.update();
//            hand.pickup2Auton();
//            return false;
//        }, new SleepAction(0.3), telemetryPacket -> {
//            telemetry.addLine("Picking up 2");
//            telemetry.update();
//            hand.open();
//            return false;
//
//        }, new SleepAction(0.7), telemetryPacket -> {
//            telemetry.addLine("Transferring 1");
//            telemetry.update();
//            hand.transfer1();
//            return false;
//        }, telemetryPacket -> {
//            telemetry.addLine(" Set Turret position to center");
//            telemetry.update();
//            hand.turrSet0Auton();
//            return false;
//        }, telemetryPacket -> {
//            telemetry.addLine("Transferring 1.5");
//            telemetry.update();
//            hand.transfer1point5();
//            return false;
//        }, telemetryPacket -> {
//            telemetry.addLine("Transferring 2");
//            telemetry.update();
//            hand.autonTransfer2();
//            return false;
//        }, new SleepAction(0.7), telemetryPacket -> {
//            telemetry.addLine("Loiter 1");
//            telemetry.update();
//            out.loiter1();
//            return false;
//        }, new SleepAction(0.3), telemetryPacket -> {
//            telemetry.addLine("Loiter 2");
//            telemetry.update();
//            out.loiter2();
//            return false;
//        }, new SleepAction(0.3), telemetryPacket -> {
//            telemetry.addLine("Loiter 3");
//            telemetry.update();
//            out.loiter3();
//            return false;
//        }, new SleepAction(0.7), telemetryPacket -> {
//            telemetry.addLine("Transfer 1");
//            telemetry.update();
//            out.transfer1();
//            return false;
//        }, new SleepAction(0.3), telemetryPacket -> {
//            telemetry.addLine("Transfer 2");
//            telemetry.update();
//            out.transfer2();
//            return false;
//        }, new SleepAction(0.3), telemetryPacket -> {
//            telemetry.addLine("Hand released sample");
//            telemetry.update();
//            hand.close();
//            return false;
//        });
//        Action basket2Action = new SequentialAction(highChamberTraj2, new SleepAction(0.3), telemetryPacket -> {
//            telemetry.addLine("Back 1");
//            telemetry.update();
//            out.back1();
//            return false;
//        }, telemetryPacket -> {
//            telemetry.addLine("Back 2");
//            telemetry.update();
//            out.back2Auton();
//            return false;
//        }, new SleepAction(0.7), telemetryPacket -> {
////            out.PIDLoop(1000);
//            telemetry.addData("Slides current pos ", out.currentPos);
//            telemetry.update();
//            return false;
//        }, new SleepAction(0.7), telemetryPacket -> {
//            telemetry.addLine("Back 2");
//            telemetry.update();
//            out.backAuton();
//            return false;
//        }, new SleepAction(0.7), telemetryPacket -> {
//            telemetry.addLine("Score");
//            telemetry.update();
//            out.score();
//            return false;
//        }, telemetryPacket -> {
//            telemetry.addLine("Set slides power to 0");
//            telemetry.update();
//            out.SlidesBrake();
//            return false;
//        }, new SleepAction(0.3), telemetryPacket -> {
//            telemetry.addLine("back 1 after placing");
//            telemetry.update();
//            out.back1();
//            return false;
//        }, new SleepAction(0.7), telemetryPacket -> {
//            telemetry.addLine("Slides down");
//            telemetry.update();
////            out.PIDLoop(0);
//            return false;
//        }, new SleepAction(0.7), telemetryPacket -> {
//            telemetry.addLine("Slides brake");
//            telemetry.update();
//            out.SlidesBrake();
//            return false;
//        });
//        Action sample2Action = new SequentialAction(sample2traj, telemetryPacket -> {
//            telemetry.addLine("Scan 1");
//            telemetry.update();
//            hand.scan1();
//            return false;
//        }, new SleepAction(0.2), telemetryPacket -> {
//            telemetry.addLine("Scan 2");
//            telemetry.update();
//            hand.scan2();
//            return false;
//        }, new SleepAction(0.2), telemetryPacket -> {
//            telemetry.addLine("Scan 3");
//            telemetry.update();
//            //hand.scan3();
//            hand.close();
//            return false;
//        }, new SleepAction(0.3), telemetryPacket -> {
//            telemetry.addLine("Hovering");
//            telemetry.update();
//            hand.hoverAuto();
//            return false;
//        }, new SleepAction(0.4), telemetryPacket -> {
//            telemetry.addLine("Picking up 1");
//            telemetry.update();
//            hand.autonPickup1();
//            return false;
//        }, new SleepAction(0.5), telemetryPacket -> {
//            telemetry.addLine("Picking up 1");
//            telemetry.update();
//            hand.pickup2Auton();
//            return false;
//        }, new SleepAction(0.3), telemetryPacket -> {
//            telemetry.addLine("Picking up 2");
//            telemetry.update();
//            hand.open();
//            return false;
//
//        }, new SleepAction(0.7), telemetryPacket -> {
//            telemetry.addLine("Transferring 1");
//            telemetry.update();
//            hand.transfer1();
//            return false;
//        }, telemetryPacket -> {
//            telemetry.addLine(" Set Turret position to center");
//            telemetry.update();
//            hand.turrSet0Auton();
//            return false;
//        }, telemetryPacket -> {
//            telemetry.addLine("Transferring 1.5");
//            telemetry.update();
//            hand.transfer1point5();
//            return false;
//        }, telemetryPacket -> {
//            telemetry.addLine("Transferring 2");
//            telemetry.update();
//            hand.autonTransfer2();
//            return false;
//        }, new SleepAction(0.7), telemetryPacket -> {
//            telemetry.addLine("Loiter 1");
//            telemetry.update();
//            out.loiter1();
//            return false;
//        }, new SleepAction(0.3), telemetryPacket -> {
//            telemetry.addLine("Loiter 2");
//            telemetry.update();
//            out.loiter2();
//            return false;
//        }, new SleepAction(0.3), telemetryPacket -> {
//            telemetry.addLine("Loiter 3");
//            telemetry.update();
//            out.loiter3();
//            return false;
//        }, new SleepAction(0.7), telemetryPacket -> {
//            telemetry.addLine("Transfer 1");
//            telemetry.update();
//            out.transfer1();
//            return false;
//        }, new SleepAction(0.3), telemetryPacket -> {
//            telemetry.addLine("Transfer 2");
//            telemetry.update();
//            out.transfer2();
//            return false;
//        }, new SleepAction(0.3), telemetryPacket -> {
//            telemetry.addLine("Hand released sample");
//            telemetry.update();
//            hand.close();
//            return false;
//        });
//        Action basket3Action = new SequentialAction(basket3traj, new SleepAction(0.3), telemetryPacket -> {
//            telemetry.addLine("Back 1");
//            telemetry.update();
//            out.back1();
//            return false;
//        }, telemetryPacket -> {
//            telemetry.addLine("Back 2");
//            telemetry.update();
//            out.back2Auton();
//            return false;
//        }, new SleepAction(0.7), telemetryPacket -> {
////            out.PIDLoop(1000);
//            telemetry.addData("Slides current pos ", out.currentPos);
//            telemetry.update();
//            return false;
//        }, new SleepAction(0.7), telemetryPacket -> {
//            telemetry.addLine("Back 2");
//            telemetry.update();
//            out.backAuton();
//            return false;
//        }, new SleepAction(0.7), telemetryPacket -> {
//            telemetry.addLine("Score");
//            telemetry.update();
//            out.score();
//            return false;
//        }, telemetryPacket -> {
//            telemetry.addLine("Set slides power to 0");
//            telemetry.update();
//            out.SlidesBrake();
//            return false;
//        }, new SleepAction(0.3), telemetryPacket -> {
//            telemetry.addLine("back 1 after placing");
//            telemetry.update();
//            out.back1();
//            return false;
//        }, new SleepAction(0.7), telemetryPacket -> {
//            telemetry.addLine("Slides down");
//            telemetry.update();
////            out.PIDLoop(0);
//            return false;
//        }, new SleepAction(0.7), telemetryPacket -> {
//            telemetry.addLine("Slides brake");
//            telemetry.update();
//            out.SlidesBrake();
//            return false;
//        });
//        Action sample3Action = new SequentialAction(sample3traj, telemetryPacket -> {
//            telemetry.addLine("Scan 1");
//            telemetry.update();
//            hand.scan1();
//            return false;
//        }, new SleepAction(0.2), telemetryPacket -> {
//            telemetry.addLine("Scan 2");
//            telemetry.update();
//            hand.scan2();
//            return false;
//        }, new SleepAction(0.2), telemetryPacket -> {
//            telemetry.addLine("Scan 3");
//            telemetry.update();
//            //hand.scan3();
//            hand.close();
//            return false;
//        }, new SleepAction(0.3), telemetryPacket -> {
//            telemetry.addLine("Hovering");
//            telemetry.update();
//            hand.hoverAuto();
//            return false;
//        }, new SleepAction(0.4), telemetryPacket -> {
//            telemetry.addLine("Picking up 1");
//            telemetry.update();
//            hand.autonPickup1();
//            return false;
//        }, new SleepAction(0.5), telemetryPacket -> {
//            telemetry.addLine("Picking up 1");
//            telemetry.update();
//            hand.pickup2Auton();
//            return false;
//        }, new SleepAction(0.3), telemetryPacket -> {
//            telemetry.addLine("Picking up 2");
//            telemetry.update();
//            hand.open();
//            return false;
//
//        }, new SleepAction(0.7), telemetryPacket -> {
//            telemetry.addLine("Transferring 1");
//            telemetry.update();
//            hand.transfer1();
//            return false;
//        }, telemetryPacket -> {
//            telemetry.addLine(" Set Turret position to center");
//            telemetry.update();
//            hand.turrSet0Auton();
//            return false;
//        }, telemetryPacket -> {
//            telemetry.addLine("Transferring 1.5");
//            telemetry.update();
//            hand.transfer1point5();
//            return false;
//        }, telemetryPacket -> {
//            telemetry.addLine("Transferring 2");
//            telemetry.update();
//            hand.autonTransfer2();
//            return false;
//        }, new SleepAction(0.7), telemetryPacket -> {
//            telemetry.addLine("Loiter 1");
//            telemetry.update();
//            out.loiter1();
//            return false;
//        }, new SleepAction(0.3), telemetryPacket -> {
//            telemetry.addLine("Loiter 2");
//            telemetry.update();
//            out.loiter2();
//            return false;
//        }, new SleepAction(0.3), telemetryPacket -> {
//            telemetry.addLine("Loiter 3");
//            telemetry.update();
//            out.loiter3();
//            return false;
//        }, new SleepAction(0.7), telemetryPacket -> {
//            telemetry.addLine("Transfer 1");
//            telemetry.update();
//            out.transfer1();
//            return false;
//        }, new SleepAction(0.3), telemetryPacket -> {
//            telemetry.addLine("Transfer 2");
//            telemetry.update();
//            out.transfer2();
//            return false;
//        }, new SleepAction(0.3), telemetryPacket -> {
//            telemetry.addLine("Hand released sample");
//            telemetry.update();
//            hand.close();
//            return false;
//        });
//        Action basket4Action = new SequentialAction(parkTraj, new SleepAction(0.3), telemetryPacket -> {
//            telemetry.addLine("Back 1");
//            telemetry.update();
//            out.back1();
//            return false;
//        }, telemetryPacket -> {
//            telemetry.addLine("Back 2");
//            telemetry.update();
//            out.back2Auton();
//            return false;
//        }, new SleepAction(0.7), telemetryPacket -> {
////            out.PIDLoop(1000);
//            telemetry.addData("Slides current pos ", out.currentPos);
//            telemetry.update();
//            return false;
//        }, new SleepAction(0.7), telemetryPacket -> {
//            telemetry.addLine("Back 2");
//            telemetry.update();
//            out.backAuton();
//            return false;
//        }, new SleepAction(0.7), telemetryPacket -> {
//            telemetry.addLine("Score");
//            telemetry.update();
//            out.score();
//            return false;
//        }, telemetryPacket -> {
//            telemetry.addLine("Set slides power to 0");
//            telemetry.update();
//            out.SlidesBrake();
//            return false;
//        }, new SleepAction(0.3), telemetryPacket -> {
//            telemetry.addLine("back 1 after placing");
//            telemetry.update();
//            out.back1();
//            return false;
//        }, new SleepAction(0.7), telemetryPacket -> {
//            telemetry.addLine("Slides down");
//            telemetry.update();
////            out.PIDLoop(0);
//            return false;
//        }, new SleepAction(0.7), telemetryPacket -> {
//            telemetry.addLine("Slides brake");
//            telemetry.update();
//            out.SlidesBrake();
//            return false;
//        });

        Actions.runBlocking(new SequentialAction(
                highChamberTraj,
                new SleepAction(3),
                obsZoneTraj,
                new SleepAction(3),
                obsZonePickupTraj,
                highChamberTraj2,
                new SleepAction(3),
                obsZoneTraj,
                obsZonePickupTraj,
                new SleepAction(3),
                highChamberTraj2,
                new SleepAction(3),
                obsZoneTraj,
                obsZonePickupTraj,
                new SleepAction(3),
                highChamberTraj2,
                parkTraj
        ));
    }
}

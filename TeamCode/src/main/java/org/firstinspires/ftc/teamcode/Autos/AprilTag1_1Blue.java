package org.firstinspires.ftc.teamcode.Autos;

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
@Autonomous(name = "April Tag Blue 1 Parking")
public class AprilTag1_1Blue extends LinearOpMode {
    public static double turnGain = 0.04;
    public static double translateGain = 0.2;
    public static double strafeGain = 0.01;

    public static double RandomdistanceUnits = 32.0;

    public static double SlideTicks = 2300;

    Drivetrain drive = new Drivetrain();
    EnableHand intake = new EnableHand();

    ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    MiggyUnLimbetedOuttake outake = new MiggyUnLimbetedOuttake();

    @Override
    public void runOpMode() throws InterruptedException {
         MecanumDrive drive2 = new MecanumDrive(hardwareMap, new Pose2d(0, 31, Math.toRadians(90)));
        intake.init(hardwareMap);
        drive.init(hardwareMap);
        outake.init(hardwareMap);
        intake.setSwingArmAngleAuton(90);
        outake.autonInit();

        Action myTrajectory = drive2.actionBuilder(drive2.pose)
                .splineToLinearHeading(new Pose2d(-37, 33, Math.toRadians(90)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-26, 9, Math.toRadians(90)), Math.toRadians(0))
                .build();

        while (!isStopRequested() && !opModeIsActive()) {
        }
        waitForStart();

        if (isStopRequested()) return;

        //moving back to see april tags
        drive.drive(700, -0.5);
        time.reset();
        time.startTime();

        while(opModeIsActive()) {

            drive.turnGain = turnGain;
            drive.translateGain = translateGain;
            drive.strafeGain = strafeGain;
            if (time.seconds()<5){
                outake.PIDLoop(SlideTicks);
                drive.alignAprilTag(RandomdistanceUnits);
                outake.back1();
                outake.back2();
            }
            else if (time.seconds()<6){
                outake.PIDLoop(SlideTicks-700);
            }
            else if (time.seconds()<8){
                outake.score();
            }
            else if(time.seconds()<9){
                outake.PIDLoop(100);
                Actions.runBlocking(new SequentialAction(myTrajectory));
            }
            else if(time.seconds()<10){
                outake.SlidesBrake();
            }



        }
    }
}


package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystem.EnableHand;

@Config
@TeleOp(name = "Adi runner Tuner")
public class AdiRunnerTuner extends LinearOpMode {
    public static double KpVert = 0.000101, KpStraffe = 0.0003, KpRotation = 0.04;
    public static double KdVertical = 0.00035,KdStrafee =0.001;
    //kpVert 0.0001
    private Drivetrain drive;

    private EnableHand hand;

    public static double Ypos = 0.0,Xpos = 0.0 ,Heading= 0.0;

    MultipleTelemetry tele;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        Init(); //fix the imu

        waitForStart();



        while (opModeIsActive()){
            drive.KpVertical = KpVert;
            drive.KpStraffe = KpStraffe;
            drive.KpRotation = KpRotation;
            drive.KdVertical = KdVertical;
            drive.KdStrafee = KdStrafee;

           double[] current =  drive.toPoint(Ypos,Xpos,Heading);

           tele.addData("Vertical Target" , Ypos);
           tele.addData("Horizontal pos", Xpos);
           tele.addData("Target heading", Heading);

           tele.addData("Current Vertical", current[0]);
           tele.addData("Current Horizontal", current[1]);
           tele.addData("Current Heading", current[2]);

           tele.update();

        }



    }

    public void Init(){


        drive = new Drivetrain();
        drive.init(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        tele =  new MultipleTelemetry(telemetry,dashboard.getTelemetry());
        hand = new EnableHand();
        hand.init(hardwareMap);
        hand.setSwingArmAngleAuton(90);
    }
}

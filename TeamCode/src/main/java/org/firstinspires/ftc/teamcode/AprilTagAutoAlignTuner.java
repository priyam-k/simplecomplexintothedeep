
package org.firstinspires.ftc.teamcode;

import android.transition.Slide;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystem.EnableHand;
import org.firstinspires.ftc.teamcode.Subsystem.MiggyUnLimbetedOuttake;

@Config
@Autonomous(name = "April Tag Tuner")

public class AprilTagAutoAlignTuner extends LinearOpMode {
    public static double  RandomdistanceUnits = 32.0;
    Drivetrain drive = new Drivetrain();
    EnableHand intake = new EnableHand();
    MiggyUnLimbetedOuttake outake = new MiggyUnLimbetedOuttake();
    private DcMotorEx slideMotorRight;
    private DcMotorEx slideMotorLeft;
    public static double Kp, targetPos;
    public static double pos = 0.46;
    MultipleTelemetry tele;


    public void runOpMode() throws InterruptedException {
        intake.init(hardwareMap);
        drive.init(hardwareMap);
        outake.init(hardwareMap);
        outake.autonInit();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        DcMotorEx slideMotorRight = hardwareMap.get(DcMotorEx.class,"rightLift");
        Servo ArmTurr = hardwareMap.get(Servo.class, "Servo10");
        intake.setSwingArmAngleAuton(90);
        waitForStart();
        outake.back1();
        outake.back2();
        while(opModeIsActive())
        {
            ArmTurr.setPosition(pos);
            drive.alignAprilTag(RandomdistanceUnits);
            // obtain the encoder position
            double encoderPositionRight = -slideMotorRight.getCurrentPosition();
            // calculate the error
           outake.PIDLoop(targetPos);
            //tele.addData("The motor power is: ", out);
            tele.addData("Current position: ", encoderPositionRight);
            tele.addData("Target position: ", targetPos);
            tele.addData("Random Distance Units it's", RandomdistanceUnits);
            tele.update();
        }
    }
}
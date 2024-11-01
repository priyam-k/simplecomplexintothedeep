
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
    public static double  RandomdistanceUnits = 28.0;
    Drivetrain drive = new Drivetrain();
    EnableHand intake = new EnableHand();
    MiggyUnLimbetedOuttake outake = new MiggyUnLimbetedOuttake();
    private DcMotorEx slideMotorRight;
    private DcMotorEx slideMotorLeft;
    public static double KpRange = 0,KpturnGain = 0,KpstrafeGain = 0;
           //slides
    public static double targetPos;

    //second kp is 0.3
    public static double pos = 0.455;
    MultipleTelemetry tele;


    public void runOpMode() throws InterruptedException {
        intake.init(hardwareMap);
        drive.init(hardwareMap);
        outake.init(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        DcMotorEx slideMotorRight = hardwareMap.get(DcMotorEx.class,"rightLift");
        Servo ArmTurr = hardwareMap.get(Servo.class, "Servo10");
        intake.setSwingArmAngleAuton(90);


        waitForStart();
        outake.back1();
        outake.back2();
        outake.transfer2();





        //start cycling
        while(opModeIsActive())
        {
            drive.translateGain = KpRange;
            drive.turnGain =  KpturnGain ;
            drive.strafeGain = KpstrafeGain;
            //APRIL TAG STUFF
            ArmTurr.setPosition(pos);
             double[] Eror = drive.alignAprilTagtuning(RandomdistanceUnits);
            // obtain the encoder position
            tele.addData("Straffe error",Eror[0]);
            //the target is zero
            tele.addData("Range error", Eror[1]);
            //current range
            tele.addData("Random Distance Units it's", RandomdistanceUnits);
            //target range
            tele.addData("Turn error",Eror[2]);
            //the target is 0



            //SLIDES STUFF
            double encoderPositionRight = -slideMotorRight.getCurrentPosition();
            // calculate the error
           outake.PIDLoop(targetPos);
            //tele.addData("The motor power is: ", out);
            tele.addData("Slides Current position:  ", encoderPositionRight);
            tele.addData(" Slides Target position: ", targetPos);

            tele.update();
        }
    }
}
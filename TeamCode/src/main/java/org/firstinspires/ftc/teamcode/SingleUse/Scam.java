package org.firstinspires.ftc.teamcode.SingleUse;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.RamseteController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "SCAM")
public class Scam extends LinearOpMode {

    private DcMotor Rarm,Larm,SLides;

    private double Angle = 0.0,power = 0.0;

    MultipleTelemetry tele;

    FtcDashboard dash;

    public static double Kp =0.0,kcos = 0.0,targetAngle = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();
        tele = new MultipleTelemetry(telemetry,dash.getTelemetry());


        Rarm = hardwareMap.get(DcMotor.class,"ArmR");
        Larm = hardwareMap.get(DcMotor.class,"ArmL");
        SLides = hardwareMap.get(DcMotor.class,"Slides");

        Rarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SLides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Rarm.setDirection(DcMotorSimple.Direction.REVERSE);

        Rarm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Larm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SLides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()){
            tele.addData("arm ricks",Rarm.getCurrentPosition());
            tele.addData(" Target Angle" ,targetAngle);
            tele.addData("Current ", TicktoAngleConverter(Rarm.getCurrentPosition()));
            tele.addData("POWER", power);
            tele.update();

            //Pid part
            power = Kp*(targetAngle - TicktoAngleConverter(Rarm.getCurrentPosition()));

                    if (power < 0) {
                        //feed forward part
                       power +=kcos * Math.cos(Math.toRadians(TicktoAngleConverter(Rarm.getCurrentPosition())));
                    }
                    else{
                        power -=kcos*Math.cos(Math.toRadians(TicktoAngleConverter(Rarm.getCurrentPosition())));
                    }

            //setting power to both
            Rarm.setPower(power);
            Larm.setPower(power);

        }
    }


    public double TicktoAngleConverter(double tick){
       double x =  (tick-173) * (90.0/706);
        return x;


        //879

        //879 - 706
    }
    public double AngletoTicks(double TargetAngle){

        return TargetAngle*(706/90.0) - 22.0538;

    }

    //Goal number 1: Writing a pid to point for the SWing arm using feedforward

    //Goal number 2: Write a tele op functionality for the arm

}

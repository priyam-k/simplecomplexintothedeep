package org.firstinspires.ftc.teamcode.SingleUse.SlidesStuff;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
@TeleOp(name = "Slides tuner")
public class SlidesTuner extends LinearOpMode {

    private DcMotorEx slideMotorRight;
    private DcMotorEx slideMotorLeft;

    public static double Kp, targetPos; //0.1, 1400

    MultipleTelemetry tele;
    // MAX POS IS 1400 TICKS
    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        slideMotorRight = hardwareMap.get(DcMotorEx.class, "rightLift");
        slideMotorLeft = hardwareMap.get(DcMotorEx.class, "leftLift");

        slideMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slideMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        slideMotorRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slideMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        while (opModeIsActive()) {
            // obtain the encoder position
            double encoderPositionRight = slideMotorRight.getCurrentPosition();
            // calculate the error
            double error = targetPos - encoderPositionRight;
            //
            double out = -(Kp * error) ;
            //1428 max

            slideMotorRight.setPower(out);
            slideMotorLeft.setPower(out);
            tele.addData("The motor power is: ", out);
            tele.addData("Current position: ", encoderPositionRight);
            tele.addData("Target position: ", targetPos);
            tele.update();
        }
    }
}

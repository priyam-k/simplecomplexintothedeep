package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
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

public class SlidesTuner extends LinearOpMode {

    private DcMotorEx slideMotorRight;
    private DcMotorEx slideMotorLeft;

    public static double Kp, targetPos;
    // MAX POS IS 5279 TICKS
    @Override
    public void runOpMode() throws InterruptedException {
        slideMotorRight = hardwareMap.get(DcMotorEx.class, "rightLift");
        slideMotorLeft = hardwareMap.get(DcMotorEx.class, "leftLift");

        slideMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        slideMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        waitForStart();
        while (opModeIsActive()) {
            // obtain the encoder position
            double encoderPositionRight = -slideMotorRight.getCurrentPosition();
            // calculate the error
            double error = targetPos - encoderPositionRight;
            //
            double out = (Kp * error) ;

            slideMotorRight.setPower(out);
            slideMotorLeft.setPower(out);
            telemetry.addData("The motor power is: ", out);
            telemetry.update();
        }
    }
}

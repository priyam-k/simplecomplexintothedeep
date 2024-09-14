package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor LF = hardwareMap.dcMotor.get("leftFront");
        DcMotor LR = hardwareMap.dcMotor.get("leftRear");
        DcMotor RF= hardwareMap.dcMotor.get("rightFront");
        DcMotor RR = hardwareMap.dcMotor.get("rightRear");

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        RR.setDirection(DcMotorSimple.Direction.REVERSE);



        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            telemetry.addData("leftFront",LF.getCurrentPosition());
            telemetry.addData("rightFront",LR.getCurrentPosition());
            telemetry.addData("rightRear",RR.getCurrentPosition());
            telemetry.addData("rightFront",RF.getCurrentPosition());
            telemetry.update();

//Right front and left front motors encoder are reversed

            //RF is LODO
            //LF is Perp or MODO
            //LR RODO

            LF.setPower(frontLeftPower);
            LR.setPower(backLeftPower);
            RF.setPower(frontRightPower);
            RR.setPower(backRightPower);
        }
    }
}
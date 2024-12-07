package org.firstinspires.ftc.teamcode.SingleUse.SlidesStuff;

import androidx.annotation.NonNull;

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

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="SlideControl", group="Linear Opmode")
public class SlideControl extends LinearOpMode {
    public static double targetPos =0.0;

    private DcMotorEx slideMotorLeft;
    private DcMotorEx slideMotorRight;

    private boolean waspressed = false;

MultipleTelemetry tele;

    @Override
    public void runOpMode() {


        // Initialize the hardware
        //TODO: Change these configs
        slideMotorRight = hardwareMap.get(DcMotorEx.class, "rightLift");
        slideMotorLeft = hardwareMap.get(DcMotorEx.class, "leftLift");

        slideMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slideMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        slideMotorLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slideMotorRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Set motor direction if needed
        //TODO: reverse on of these
        //slideMotorRight.setDirection(DcMotorEx.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.a){
                waspressed = true;
            }
            if(!gamepad1.a && waspressed){
                slideMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slideMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slideMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slideMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                waspressed = false;
            }

                slideMotorRight.setPower(gamepad1.left_stick_y);
                slideMotorLeft.setPower(gamepad1.left_stick_y);



            // Show the motor power in telemetry
            telemetry.addData("Power",gamepad1.left_stick_y);
            telemetry.addData("Slide Motor right position ", slideMotorRight.getCurrentPosition());
            telemetry.update();

        }
    }
//    public Action slideUp() {
//        return new Action() {
//            int targetPos = 500;
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                slideMotorLeft.setPower(0.8);
//                slideMotorRight.setPower(0.8);
//                return false;
//            }
//        };
//    }
}

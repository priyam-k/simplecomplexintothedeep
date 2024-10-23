package org.firstinspires.ftc.teamcode;

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


    private DcMotorEx slideMotorLeft;
    private DcMotorEx slideMotorRight;

MultipleTelemetry tele;

    @Override
    public void runOpMode() {


        // Initialize the hardware
        //TODO: Change these configs
        slideMotorRight = hardwareMap.get(DcMotorEx.class, "rightLift");
        slideMotorLeft = hardwareMap.get(DcMotorEx.class, "leftLift");

        slideMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        slideMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        slideMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slideMotorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set motor direction if needed
        //TODO: reverse on of these
        //slideMotorRight.setDirection(DcMotorEx.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

                slideMotorRight.setPower(gamepad1.left_stick_y);
                slideMotorLeft.setPower(gamepad1.left_stick_y);

            telemetry.addData("leftLiftCurrent AMPS: ", slideMotorLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("rightLiftCurrent AMPS: ", slideMotorRight.getCurrent(CurrentUnit.AMPS));
            // Show the motor power in telemetry
            telemetry.addData("Slide Motor Power right", slideMotorRight.getPower());
            telemetry.addData("Slide Motor Power left", slideMotorLeft.getPower());
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

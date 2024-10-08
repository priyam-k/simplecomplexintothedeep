package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystem.EnableHand;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;
import org.firstinspires.ftc.teamcode.Subsystem.Robot;

@TeleOp
@Config
public class MecanumTeleOp extends LinearOpMode {
    // NEEDS TO BE TESTED

    Drivetrain drive;
    EnableHand Hand;

    public static double both = 0;
    private boolean waspressed = false;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drivetrain();
        Hand = new EnableHand();
        drive.init(hardwareMap);
        Hand.init(hardwareMap);
        Outtake outtake = new Outtake();
        outtake.setOTState(Outtake.OuttakeStates.LOITER1);

        waitForStart();

        ElapsedTime t = new ElapsedTime();
        boolean turretRightCheck = false;
        boolean turretLeftCheck = false;
        boolean handRightCheck = false;
        boolean handLeftCheck = false;


        while (opModeIsActive()){
            drive.TeleopControl(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
            Hand.BothArm(both);
            if (gamepad1.a){waspressed = true;}
            if (!gamepad1.a && waspressed){
                Hand.close();
                waspressed = false;}


            switch (outtake.getOTState()) {
                case LOITER1:
                    if (gamepad2.a) {
                        outtake.loiter1();
                        t.reset();

                        if (t.milliseconds() >= 500)
                            outtake.setOTState(Outtake.OuttakeStates.GRAB1);
                    }

                    break;

                case GRAB1:
                    if (gamepad2.a) {
                        outtake.grab1();
                        t.reset();
                        if (t.milliseconds() >= 500)
                            outtake.setOTState(Outtake.OuttakeStates.BACK1);
                    }
                    break;

                case ROTATEPOS:

                    if(gamepad2.dpad_right && !turretRightCheck){
                        outtake.incrementTurretServo(0.1);
                        turretRightCheck = true;
                    }
                    if (turretRightCheck && !gamepad2.dpad_right) {
                        turretRightCheck = false;
                    }
                    if(gamepad2.dpad_left && !turretLeftCheck){
                        outtake.incrementTurretServo(-0.1);
                        turretLeftCheck = true;
                    }
                    if (turretLeftCheck && !gamepad2.dpad_left) {
                        turretLeftCheck = false;
                    }
                    if(gamepad2.dpad_up && !handLeftCheck){
                        outtake.incrementHandTurret(0.1);
                        handLeftCheck = true;
                    }
                    if (handLeftCheck && !gamepad2.dpad_up) {
                        handLeftCheck = false;
                    }
                    if(gamepad2.dpad_down && !handRightCheck){
                        outtake.incrementHandTurret(-0.1);
                        handRightCheck = true;
                    }
                    if (handRightCheck && !gamepad2.dpad_down) {
                        handRightCheck = false;
                    }

                    if(gamepad2.a){
                        outtake.setOTState(Outtake.OuttakeStates.FINALGRAB);
                    }

                case FINALGRAB:
                    if (gamepad2.a) {
                        outtake.finalGrab();
                        t.reset();

                        if (t.milliseconds() >= 500)
                            outtake.setOTState(Outtake.OuttakeStates.BACK1);
                    }

                    break;


                case BACK1:
                    if (gamepad2.a) {
                        outtake.back1();
                        t.reset();
                        if (t.milliseconds() >= 500)
                            outtake.setOTState(Outtake.OuttakeStates.LOITER1);
                    }
                    break;
            }


        }//opmode end

    }
}
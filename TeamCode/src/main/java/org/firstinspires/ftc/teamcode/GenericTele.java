package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.sfdev.assembly.state.StateMachine;

import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystem.EnableHand;
import org.firstinspires.ftc.teamcode.Subsystem.MiggyUnLimbetedOuttake;
import org.firstinspires.ftc.teamcode.Subsystem.StateMachines;
import org.firstinspires.ftc.teamcode.Subsystem.Subsystem;

@TeleOp(name="Generic Tele")
@Config
public class GenericTele extends LinearOpMode {

    Drivetrain drive;
    public Servo slidesServo;
    EnableHand hand;
    MiggyUnLimbetedOuttake out;
    StateMachine intakeMachine,transferMachine, slidesMachine;

    public double HoldingSlide = 0.12;

    public double SlidesActivated = 0;



    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drivetrain();
        hand = new EnableHand();
        out = new MiggyUnLimbetedOuttake();
        boolean slidesbuttonpressed = false;
        slidesServo = hardwareMap.get(Servo.class, "Servo4");


        drive.init(hardwareMap);
        hand.init(hardwareMap, gamepad2);
        out.init(hardwareMap);

        intakeMachine = StateMachines.getIntakeStateMachine(hand, gamepad2, transferMachine);
        transferMachine = StateMachines.getOuttakeStateMachine(out, gamepad2, intakeMachine);
        slidesMachine = StateMachines.getSlidesStateMachine(out, gamepad2);

        waitForStart();

        transferMachine.start();
        intakeMachine.start();
        slidesMachine.start();
        telemetry.addData("rightLift power:", out.slidesPower());
        telemetry.update();
        slidesServo.setPosition(0.12);

        while (opModeIsActive()) {
            transferMachine.update();
            intakeMachine.update();
            slidesMachine.update();
            //out.Lift(gamepad2.left_stick_y);

            telemetry.addData("Intake state", intakeMachine.getStateString());
            telemetry.addData("Slides state", slidesMachine.getStateString());
            if(gamepad1.right_bumper){
                drive.TeleopControl(gamepad1.left_stick_y*0.7,gamepad1.left_stick_x*0.7,gamepad1.right_stick_x/2.0);
            }
            else{
                drive.TeleopControl(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
            }
            if (!gamepad2.y && slidesbuttonpressed){
               double x =  (slidesServo.getPosition() == SlidesActivated)? HoldingSlide : SlidesActivated;
               slidesServo.setPosition(x);
            slidesbuttonpressed = false;}

            if (gamepad2.y){
                slidesbuttonpressed = true;
            }
            telemetry.addData("rightLift power:", out.slidesPower());








            // out.liftSetPos(gamepad2); hanging stuff

            telemetry.update();
        }

    }
}
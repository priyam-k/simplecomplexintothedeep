package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;

import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystem.EnableHand;
import org.firstinspires.ftc.teamcode.Subsystem.MiggyUnLimbetedOuttake;
import org.firstinspires.ftc.teamcode.Subsystem.StateMachines;

@TeleOp(name = "Generic Tele")
@Config
public class GenericTele extends LinearOpMode {

    public double HoldingSlide = 0.12;
    public double SlidesActivated = 0;
    Drivetrain drive;
    EnableHand hand;
    MiggyUnLimbetedOuttake out;
    StateMachine intakeMachine, transferMachine;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drivetrain();
        hand = new EnableHand();
        out = new MiggyUnLimbetedOuttake();


        drive.init(hardwareMap);
        hand.init(hardwareMap, gamepad2);
        hand.init(hardwareMap, gamepad2);
        out.init(hardwareMap);

        intakeMachine = StateMachines.getIntakeStateMachine(hand, gamepad2);
        transferMachine = StateMachines.getOuttakeStateMachine(out, gamepad2, intakeMachine, telemetry);

        waitForStart();

        transferMachine.start();
        intakeMachine.start();
        telemetry.addData("rightLift power:", out.slidesPower());
        telemetry.update();


        while (opModeIsActive()) {
            transferMachine.update();
            intakeMachine.update();
            if (intakeMachine.getStateString() == "HOVERING" || intakeMachine.getStateString() == "PICKUP2" || transferMachine.getStateString() == "BACK1HIGH") {
                out.Lift(gamepad2.left_stick_y);
            }


            telemetry.addData("Intake state", intakeMachine.getStateString());
            telemetry.addData("Outtake States:", transferMachine.getStateString());
            if (gamepad1.right_bumper) {
                drive.TeleopControl(gamepad1.left_stick_y * 0.7, gamepad1.left_stick_x * 0.7, gamepad1.right_stick_x / 2.0);
            } else {
                drive.TeleopControl(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }


            // out.liftSetPos(gamepad2); hanging stuff

            telemetry.update();
        }

    }
}
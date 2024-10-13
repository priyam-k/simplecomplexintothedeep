package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystem.EnableHand;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

@TeleOp(name="Generic Tele")
@Config
public class GenericTele extends LinearOpMode {

    Drivetrain drive;
    EnableHand hand;
    Outtake out;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drivetrain();
        hand = new EnableHand();
        out = new Outtake();

        drive.init(hardwareMap);
        hand.init(hardwareMap);
        out.init(hardwareMap);

        StateMachine transferMachine = StateMachines.getOuttakeStateMachine(out);

        waitForStart();


        transferMachine.start();


        while (opModeIsActive()) {
            //transferMachine.update();
            drive.TeleopControl(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad2.right_stick_x);

        }

    }
}
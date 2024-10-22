package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;

import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystem.*;

@TeleOp(name="GenericAuto")
public class Auto extends LinearOpMode {

    public static double turnGain = 0.03;
    public static double translateGain = 0.05;
    public static double strafeGain = 0.03;

    Drivetrain drive;
    EnableHand intake;
    MiggyUnLimbetedOuttake outtake;


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drivetrain();
        intake = new EnableHand();
        outtake = new MiggyUnLimbetedOuttake();

        drive.init(hardwareMap);
        intake.init(hardwareMap);
        outtake.init(hardwareMap);

        StateMachine intakeMachine = StateMachines.getIntakeStateMachine(intake, gamepad2);
        StateMachine transferMachine = StateMachines.getOuttakeStateMachine(outtake, gamepad2, intakeMachine);

        transferMachine.start();
        intakeMachine.start();

        while (opModeInInit()) {

            drive.turnGain = turnGain;
            drive.translateGain = translateGain;
            drive.strafeGain = strafeGain;

            drive.alignAprilTag(24);

        }

        while (opModeIsActive()){
            transferMachine.update();
            intakeMachine.update();
            telemetry.addData("Intake state", intakeMachine.getStateString());

            //drive.TeleopControl(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
            drive.drive(10);
//            drive.alignAprilTag();

            telemetry.update();
        }

        waitForStart();


    }
}

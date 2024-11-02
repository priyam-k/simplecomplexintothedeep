package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;

import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystem.*;

@TeleOp(name="GenericAuto")
public class Auto extends LinearOpMode {

//    public static double turnGain = 0.03;
//    public static double translateGain = 0.05;
//    public static double strafeGain = 0.03;

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

        StateMachine machine = StateMachines.getMachine(outtake, intake, gamepad2);

        machine.start();


//        while (opModeInInit()) {
//
//            drive.turnGain = turnGain;
//            drive.translateGain = translateGain;
//            drive.strafeGain = strafeGain;
//
//            drive.alignAprilTag(24);
//
//        }

        waitForStart();

        while (opModeIsActive()){
            machine.update();
            telemetry.addData("state", machine.getStateString());

            //drive.TeleopControl(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
          //  drive.drive(-0.3);
            intake.setSwingArmAngle(90);
            //drive.alignAprilTag(15, 12);

//            intakeMachine.setState(StateMachines.Intake.SCANNING1);


            telemetry.update();
        }

        waitForStart();


    }
}

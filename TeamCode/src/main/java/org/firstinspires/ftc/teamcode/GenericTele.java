package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.sfdev.assembly.state.StateMachine;

import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystem.EnableHand;
import org.firstinspires.ftc.teamcode.Subsystem.MiggyUnLimbetedOuttake;
import org.firstinspires.ftc.teamcode.Subsystem.StateMachines;

@TeleOp(name = "Generic Tele")
@Config
public class GenericTele extends LinearOpMode {

    Drivetrain drive;
    EnableHand hand;
    MiggyUnLimbetedOuttake out;


    private DcMotorEx slideMotorLeft;
    private DcMotorEx slideMotorRight;


    public void slidesControl() {
        slideMotorRight = hardwareMap.get(DcMotorEx.class, "rightLift");
        slideMotorLeft = hardwareMap.get(DcMotorEx.class, "leftLift");

        slideMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slideMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        slideMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slideMotorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        slideMotorRight.setPower(gamepad2.left_stick_y);
        slideMotorLeft.setPower(gamepad2.left_stick_y);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drivetrain();
        hand = new EnableHand();
        out = new MiggyUnLimbetedOuttake();

        drive.init(hardwareMap);
        hand.init(hardwareMap);
        out.init(hardwareMap);

        StateMachine intakeMachine = StateMachines.getIntakeStateMachine(hand, gamepad2);
        StateMachine transferMachine = StateMachines.getOuttakeStateMachine(out, gamepad2, intakeMachine);


        waitForStart();

        transferMachine.start();
        intakeMachine.start();

        while (opModeIsActive()) {
            transferMachine.update();
            intakeMachine.update();

            telemetry.addData("Intake state", intakeMachine.getStateString());


            drive.TeleopControl(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            slidesControl();


            telemetry.update();
        }

    }
}
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.sfdev.assembly.state.StateMachine;

import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystem.EnableHand;
import org.firstinspires.ftc.teamcode.Subsystem.MiggyUnLimbetedOuttake;
import org.firstinspires.ftc.teamcode.Subsystem.StateMachineV2;

@TeleOp(name = "Gubernamental Tele")
@Config
public class Gubernamental extends LinearOpMode {

    public double HoldingSlide = 0.12;
    public double SlidesActivated = 0;
    Drivetrain drive;
    EnableHand hand;
    MiggyUnLimbetedOuttake out;
    boolean check = false;
    boolean SampleMode = true;


    private DigitalChannel redLED;
    private DigitalChannel greenLED;



    private DigitalChannel redLED2;
    private DigitalChannel greenLED2;

    StateMachine intakeMachine, sampleMachine, specimenMachine;

    @Override
    public void runOpMode() throws InterruptedException {
        redLED = hardwareMap.get(DigitalChannel.class, "red");
        greenLED = hardwareMap.get(DigitalChannel.class, "green");

        redLED2 = hardwareMap.get(DigitalChannel.class, "red2");
        greenLED2 = hardwareMap.get(DigitalChannel.class, "green2");


        drive = new Drivetrain();
        hand = new EnableHand();
        out = new MiggyUnLimbetedOuttake();


        drive.init(hardwareMap);
        hand.init(hardwareMap, gamepad2);
        hand.init(hardwareMap, gamepad2);
        out.init(hardwareMap);

        intakeMachine = StateMachineV2.getIntakeStateMachine(hand, gamepad2);
        sampleMachine = StateMachineV2.getOuttakeStateMachine(out, gamepad2, intakeMachine);
        specimenMachine = StateMachineV2.getSpecimenStateMachine(hand, out, gamepad2, telemetry);

        waitForStart();

        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
        redLED2.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED2.setMode(DigitalChannel.Mode.OUTPUT);

        sampleMachine.start();
        intakeMachine.start();
        telemetry.addData("rightLift power:", out.slidesPower());
        telemetry.update();


        while (opModeIsActive()) {

                sampleMachine.update();
                specimenMachine.update();
                intakeMachine.update();




            out.Lift(gamepad2.left_stick_y);//full manuel
            if (SampleMode) {
              //  out.Lift(gamepad2.left_stick_y);
            }

            if (gamepad2.left_bumper) {
                check = true;
            }
            if (!gamepad2.left_bumper && check) {
                check = false;
                if (!SampleMode) {

                    redLED.setState(false);
                    redLED2.setState(false);
                    greenLED.setState(true);
                    greenLED2.setState(true);

                    specimenMachine.stop();

                    sampleMachine.reset();
                    intakeMachine.reset();

                    intakeMachine.start();
                    sampleMachine.start();
                    telemetry.addLine("Stopped specimen, started sample");
                    gamepad2.rumble(100);


                } else {

                    redLED.setState(true);
                    redLED2.setState(true);

                    greenLED.setState(false);
                    greenLED2.setState(false);

                    sampleMachine.stop();
                    intakeMachine.stop();

                    specimenMachine.reset();

                    specimenMachine.start();
                    telemetry.addLine("Stopped sample, started specimen");
                    gamepad2.rumble(100);
                }

                SampleMode = !SampleMode;
            }


            telemetry.addData("Intake state", intakeMachine.getStateString());
            telemetry.addData("Outtake States:", sampleMachine.getStateString());
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


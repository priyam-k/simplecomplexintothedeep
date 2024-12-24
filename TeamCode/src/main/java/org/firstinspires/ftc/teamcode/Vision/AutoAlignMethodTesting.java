package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystem.Robot;

@Config
@TeleOp(name = "AutoAlignTest")
public class AutoAlignMethodTesting extends LinearOpMode {

    private Robot robo;
    @Override
    public void runOpMode() throws InterruptedException {

        robo = new Robot();

        robo.init(hardwareMap);
        waitForStart();

        robo.AutoAlign();

        robo.pickUp();





        while(opModeIsActive()){

        }
    }

}

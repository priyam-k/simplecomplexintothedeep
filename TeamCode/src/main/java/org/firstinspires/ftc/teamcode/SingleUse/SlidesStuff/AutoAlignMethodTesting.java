package org.firstinspires.ftc.teamcode.SingleUse.SlidesStuff;

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
    robo.initAutoAlign();
        waitForStart();

        robo.AutoAlign();




        while(opModeIsActive()){

        }
    }

}

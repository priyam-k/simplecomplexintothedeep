package org.firstinspires.ftc.teamcode.SingleUse;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystem.EnableHand;
import org.firstinspires.ftc.teamcode.Subsystem.MiggyUnLimbetedOuttake;


public class outTester extends LinearOpMode{
    MiggyUnLimbetedOuttake out;
    @Override
    public void runOpMode() throws InterruptedException {
        out = new MiggyUnLimbetedOuttake();
        out.init(hardwareMap);
        while(!isStopRequested() && !opModeIsActive()){}
        waitForStart();
        while (opModeIsActive()) {
            if (isStopRequested()) return;
            out.loiter1();
            out.transfer2();
            out.loiter3();
        }
    }
}

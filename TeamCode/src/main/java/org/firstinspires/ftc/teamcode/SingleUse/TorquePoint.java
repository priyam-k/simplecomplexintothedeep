package org.firstinspires.ftc.teamcode.SingleUse;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;

@TeleOp(name = "Torque graph")
public class TorquePoint extends LinearOpMode {
    Drivetrain train;
    ElapsedTime t;

    double avgRF,avgLF,avgRR,avgRL,counter;

    @Override
    public void runOpMode() throws InterruptedException {
        train = new Drivetrain();
        train.init(hardwareMap);

        t = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        waitForStart();
        t.reset();
        t.startTime();

        while (opModeIsActive()) {

            if (t.seconds() < 0.5) {
                train.drive(1.0);
            } else if (t.seconds() < 1.5) {

                double[] ary = train.getCurrent();

                String RF = "Right Front: " + ary[0];
                avgRF += ary[0];

                String LF = "Left Front: " + ary[1];
                avgLF += ary[1];

                String RR = "Right Rear: " + ary[2];
                avgRR += ary[2];

                String RL = "Right Left: " + ary[3];
                avgRL += ary[3];

                telemetry.addLine(RF);
                telemetry.addLine(LF);
                telemetry.addLine(RR);
                telemetry.addLine(RL);
                telemetry.update();

                counter++;

            } else if (t.seconds() < 1.51) {
                train.Brake();

                String RF = "Right Front average: " + avgRF/counter;

                String LF = "Left Front: " + avgLF/counter;

                String RR = "Right Rear: " + avgRR/counter;

                String RL = "Right Left: " + avgRL/counter;

                telemetry.addLine(RF);
                telemetry.addLine(LF);
                telemetry.addLine(RR);
                telemetry.addLine(RL);
                telemetry.update();

            }


        }
    }
}

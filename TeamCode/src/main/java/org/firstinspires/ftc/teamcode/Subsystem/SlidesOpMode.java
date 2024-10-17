package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SlidesOpMode implements Subsystem {
    MultipleTelemetry tele;
    DcMotorEx LeftLift,RightLift;
    @Override
    public void init(HardwareMap hardwareMap) { //TODO SET pid TUNING AND also CONFIG ON PHONE
       double leftLiftCurrentPos;
        double rightLiftCurrentPos;


        FtcDashboard dashboard = FtcDashboard.getInstance();
        tele = new MultipleTelemetry(tele, dashboard.getTelemetry());

        LeftLift = hardwareMap.get(DcMotorEx.class, "LFD");
        RightLift = hardwareMap.get(DcMotorEx.class, "LRD");

        // Reset motor power
        LeftLift.setPower(0.0);
        RightLift.setPower(0.0);

        // Set motor direction
        // TODO TEST OR IT WILL BREAK!!!! dry run maybe
        // LeftLift.setDirection(DcMotor.Direction.REVERSE);
        // RightLift.setDirection(DcMotor.Direction.FORWARD);


        // Set motor mode to RUN_USING_ENCODER for localization purposes
        LeftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set motor to brake/lock the wheels when there is no power being applied
        LeftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLiftCurrentPos= LeftLift.getCurrentPosition();
        rightLiftCurrentPos= RightLift.getCurrentPosition();

        tele.addData("LeftliftCurrentPos: ", leftLiftCurrentPos);
        tele.addData("RightliftCurrentPos: ", rightLiftCurrentPos);

    }
    public double getCurrentVal(DcMotorEx motor) {
        return motor.getCurrentPosition();
    }
    @Override
    public void update() {

    }

    @Override
    public void addTelemtry(Telemetry t) {

    }
}

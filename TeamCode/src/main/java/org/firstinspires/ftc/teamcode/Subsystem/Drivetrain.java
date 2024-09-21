package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivetrain implements  Subsystem{

    private DcMotor LF,LR,RF,RR;
    private Gamepad g1,g2;

    private Telemetry t;

    @Override
    public void init(HardwareMap hardwareMap, Gamepad g1, Gamepad g2) {

        LF = hardwareMap.dcMotor.get("leftFront");
        LR = hardwareMap.dcMotor.get("leftRear");
        RF= hardwareMap.dcMotor.get("rightFront");
        RR = hardwareMap.dcMotor.get("rightRear");

        //this must come before the run without encoder
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        RR.setDirection(DcMotorSimple.Direction.REVERSE);

        g1 = new Gamepad();
        g2 = new Gamepad();
    }

    public void TeleopControl(){
        double y = -g1.left_stick_y; // Remember, Y stick value is reversed
        double x = g1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = g2.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;


//Right front and left front motors encoder are reversed

        //RF is LODO
        //LF is Perp or MODO
        //LR RODO

        LF.setPower(frontLeftPower);
        LR.setPower(backLeftPower);
        RF.setPower(frontRightPower);
        RR.setPower(backRightPower);
    }

    public void getOdometeryValues(){
        t.addData("Perpendicular or leftFront",LF.getCurrentPosition());
        t.addData("Rodo on leftRront",LR.getCurrentPosition());
        t.addData("Lodo on rightFront",RF.getCurrentPosition());
        t.update();
    }


    @Override
    public void update() {

    }

    @Override
    public void addTelemtry(Telemetry t) {

    }
}

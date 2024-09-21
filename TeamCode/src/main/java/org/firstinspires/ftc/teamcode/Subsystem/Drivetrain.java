package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Drivetrain implements  Subsystem{

    private DcMotor LF,LR,RF,RR;

    private AprilTagProcessor aprilTag;



    private VisionPortal VP;
    private Telemetry t;

    private double CameraDistancefromCenter = 6;


    @Override
    public void init(HardwareMap hardwareMap) {

        aprilTag = new AprilTagProcessor.Builder().build();

        VisionPortal VP = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

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

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        RR.setDirection(DcMotorSimple.Direction.REVERSE);

    }


    public void alignAprilTag(double distance)
    public void TeleopControl(double y, double x, double rx){
        y = -y; // Remember, Y stick value is reversed
        y = Math.pow(y,3);
       x = Math.pow(x,3);
        rx = rx*0.75;




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

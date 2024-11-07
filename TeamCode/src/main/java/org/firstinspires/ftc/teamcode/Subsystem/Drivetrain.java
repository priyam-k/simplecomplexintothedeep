package org.firstinspires.ftc.teamcode.Subsystem;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.opencv.core.Point;

import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class Drivetrain implements Subsystem {

    public static double turnGain = 0.03;
    public static double translateGain = 0.08;
    // Approx: 0.8 and exact: 0.3
    public static double strafeGain = 0.015;

    public static double KpVertical = 0.0,KpStraffe = 0.0,KpRotation = 0.0;

    public static double StrafeLine = 320;  //640

    public static double VerticalLine = 240; //480


    private DcMotor LF, LR, RF, RR,par,perp;

    //TODO: tune this on the new robot
    private AprilTagProcessor aprilTag;

    private AprilTagDetection desiredTag = null;

    private VisionPortal VP;
    private Telemetry t;
    private IMU imu;
    private YawPitchRollAngles angles;



    @Override
    public void init(HardwareMap hardwareMap) {

        aprilTag = new AprilTagProcessor.Builder().build();

        VisionPortal VP = new VisionPortal.Builder().setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).addProcessor(aprilTag).build();

        LF = hardwareMap.dcMotor.get("leftFront");
        LR = hardwareMap.dcMotor.get("leftRear");
        RF = hardwareMap.dcMotor.get("rightFront");
        RR = hardwareMap.dcMotor.get("rightRear");

        par = hardwareMap.get(DcMotorEx.class, "Rodo");
        perp = hardwareMap.get(DcMotorEx.class, "Lodo");

        par.setDirection(DcMotorSimple.Direction.REVERSE);


        //this must come before the run without encoder
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        par.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        par.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        perp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // correct motor directions for Dory
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LR.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));
        //TODO FIX THIS PART
        imu.resetYaw();

    }

    public void SampleAlign(Point centerofSample){
       double x = centerofSample.x;
       double y = centerofSample.y;

            //make variables for all errors (for rotate, translate, and strafe)
            double TranslateError = VerticalLine - y; // positive error -> robot needs to move right
            double StraffeError = StrafeLine - x;// positive error -> robot needs to move forward


            //using PID to align robot to the april tag

            double drive = Range.clip(StraffeError * translateGain, -1, 1);
            double strafe = Range.clip(TranslateError * strafeGain, -1, 1);


            //calculate the powers for all motors
            double leftFrontPower = +strafe + drive;//-turn
            double rightFrontPower = -strafe + drive;//+
            double leftBackPower = -strafe + drive;//-
            double rightBackPower = +strafe + drive;//




            //setting power to all motors
            LF.setPower(leftFrontPower);
            RF.setPower(rightFrontPower);
            LR.setPower(leftBackPower);
            RR.setPower(rightBackPower);



    }


    public double[] alignAprilTagtuning(double distance) {
        double[] CurrentPosition = new double[3];

        //make an ArrayList to store the pose values for the april tags detected
        List<Double> yValues = new ArrayList<>();
        desiredTag = null;
        //make an ArrayList to store the april tags detected
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // Extract y values from detections
        for (AprilTagDetection detection : currentDetections) {
            yValues.add(detection.ftcPose.y);
        }
        //check if the ArrayList containing the pose values for the april tags is empty and if the first element is null
        //if true, find the nearest april tag and align the robot so that it is facing it; if false, set motors to 0 power
        if (!yValues.isEmpty() && yValues.get(0) != null) {
            double smallest_yVal = yValues.get(0);
            int smallestIndex = 0;

            // Find the detection with the smallest y value
            for (int i = 1; i < yValues.size(); i++) {
                if (yValues.get(i) < smallest_yVal) {
                    smallest_yVal = yValues.get(i);
                    smallestIndex = i;
                }
            }
            //assign the april tag that is the closest to desiredTag
            desiredTag = currentDetections.get(smallestIndex);


            //make variables for all errors (for rotate, translate, and strafe)
            double yawError = desiredTag.ftcPose.yaw; // positive error -> robot needs to move right
            double rangeError = desiredTag.ftcPose.range - distance; // positive error -> robot needs to move forward
            double headingError = desiredTag.ftcPose.bearing; // positive error -> robot needs to turn counterclockwise


            //yaw,range,bearing
            CurrentPosition[0] = yawError;
            CurrentPosition[1] = rangeError;
            CurrentPosition[2] = headingError;

            //using PID to align robot to the april tag
            double turn = Range.clip(headingError * turnGain, -1, 1);
            double drive = Range.clip(rangeError * translateGain, -1, 1);
            double strafe = Range.clip(yawError * strafeGain, -1, 1);


            //calculate the powers for all motors
            double leftFrontPower = +strafe + drive - turn;
            double rightFrontPower = -strafe + drive + turn;
            double leftBackPower = -strafe + drive - turn;
            double rightBackPower = +strafe + drive + turn;



            //setting power to all motors
            LF.setPower(leftFrontPower);
            RF.setPower(rightFrontPower);
            LR.setPower(leftBackPower);
            RR.setPower(rightBackPower);

        } else {
            //if april tag is not visible, stop motor power
            LF.setPower(0);
            RF.setPower(0);
            LR.setPower(0);
            RR.setPower(0);
        }

        return CurrentPosition;
    }// April tag method end

    public void alignAprilTag(double distance) {


        //make an ArrayList to store the pose values for the april tags detected
        List<Double> yValues = new ArrayList<>();
        desiredTag = null;
        //make an ArrayList to store the april tags detected
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // Extract y values from detections
        for (AprilTagDetection detection : currentDetections) {
            yValues.add(detection.ftcPose.y);
        }
        //check if the ArrayList containing the pose values for the april tags is empty and if the first element is null
        //if true, find the nearest april tag and align the robot so that it is facing it; if false, set motors to 0 power
        if (!yValues.isEmpty() && yValues.get(0) != null) {
            double smallest_yVal = yValues.get(0);
            int smallestIndex = 0;

            // Find the detection with the smallest y value
            for (int i = 1; i < yValues.size(); i++) {
                if (yValues.get(i) < smallest_yVal) {
                    smallest_yVal = yValues.get(i);
                    smallestIndex = i;
                }
            }
            //assign the april tag that is the closest to desiredTag
            desiredTag = currentDetections.get(smallestIndex);


            //make variables for all errors (for rotate, translate, and strafe)
            double yawError = desiredTag.ftcPose.yaw; // positive error -> robot needs to move right
            double rangeError = desiredTag.ftcPose.range - distance; // positive error -> robot needs to move forward
            double headingError = desiredTag.ftcPose.bearing; // positive error -> robot needs to turn counterclockwise

            //using PID to align robot to the april tag
            double turn = Range.clip(headingError * turnGain, -1, 1);
            double drive = Range.clip(rangeError * translateGain, -1, 1);
            double strafe = Range.clip(yawError * strafeGain, -1, 1);


            //calculate the powers for all motors
            double leftFrontPower = +strafe + drive - turn;
            double rightFrontPower = -strafe + drive + turn;
            double leftBackPower = -strafe + drive - turn;
            double rightBackPower = +strafe + drive + turn;



            //setting power to all motors
            LF.setPower(leftFrontPower);
            RF.setPower(rightFrontPower);
            LR.setPower(leftBackPower);
            RR.setPower(rightBackPower);

        } else {
            //if april tag is not visible, stop motor power
            LF.setPower(0);
            RF.setPower(0);
            LR.setPower(0);
            RR.setPower(0);
        }

    }// April tag method end


    public void drive(double power) {
        // positive power is forward, negative is backward
        // Set power to all motors
        LF.setPower(power);
        LR.setPower(power);
        RF.setPower(power);
        RR.setPower(power);
    }


    public void TeleopControl(double y, double x, double rx) {
        y = -y; // Remember, Y stick value is reversed
        y = Math.pow(y, 3);
        x = Math.pow(x, 3);
        rx = rx * 0.75;


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;


        //Right front and left front motors encoder are reversed


        LF.setPower(frontLeftPower);
        LR.setPower(backLeftPower);
        RF.setPower(frontRightPower);
        RR.setPower(backRightPower);
    }

    public void getOdometeryValues() {
        t.addData("Perpendicular or leftFront", LF.getCurrentPosition());
        t.addData("Rodo on leftRront", LR.getCurrentPosition());
        t.addData("Lodo on rightFront", RF.getCurrentPosition());
        t.update();
    }

    public void Brake(){
        LF.setPower(0);
        LR.setPower(0);
        RF.setPower(0);
        RR.setPower(0);
    }



    public double[] toPoint(double targetVert,double targetHorizontal,double targetHeading){

        double VerticalError = targetVert - par.getCurrentPosition();
        double StraffeError = targetHorizontal - perp.getCurrentPosition();
        angles = imu.getRobotYawPitchRollAngles();
        double HeadingError = targetHeading - angles.getYaw(AngleUnit.DEGREES);


        double turn = Range.clip(HeadingError * KpRotation, -1, 1);
        double drive = Range.clip(VerticalError * KpVertical, -1, 1);
        double strafe = Range.clip(StraffeError * KpStraffe, -1, 1);


        //calculate the powers for all motors
        double leftFrontPower = +strafe + drive - turn;
        double rightFrontPower = -strafe + drive + turn;
        double leftBackPower = -strafe + drive - turn;
        double rightBackPower = +strafe + drive + turn;

        LF.setPower(leftFrontPower);
        RF.setPower(rightFrontPower);
        LR.setPower(leftBackPower);
        RR.setPower(rightBackPower);

        double[] current = new double[3];
        current[0] = par.getCurrentPosition();
        current[1] = perp.getCurrentPosition();
        current[2] = angles.getYaw();

        return current;

    }
    @Override
    public void update() {

    }

    @Override
    public void addTelemetry(Telemetry t) {

    }
    
}

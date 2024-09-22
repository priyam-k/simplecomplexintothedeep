package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "AprilTag DriveBase Alignment Gain Tuner")
public class AprilTagDriveBaseAlignmentGainTuner extends LinearOpMode {
    public static double lateralDistance = 12;
    // proportional control variable for turning
    public static double turnGain = 0.03;
    public static double translateGain = 0.015;
    public static double strafeGain = 0.015;
    public static double motorPowerCutOff = 0.05;
    private boolean camOn = true;
    private AprilTagDetection desiredTag = null; // Used to hold the data for a detected AprilTag

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the AprilTag processor and VisionPortal
        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder().build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        // Initialize motors
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        //set directions for motors
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        //set motor mode to RUN_WITHOUT_ENCODER for localization purposes
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set motor to brake and lock the wheels when there is no power being applied
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize dashboard telemetry
        FtcDashboard dashboard = FtcDashboard.getInstance();
        MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        while (opModeInInit()) {
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

                //use telemetry to check if the camera is on and if the april tag was spotted
                tele.addData("Camera on: ", camOn);
                tele.addData("Tag spotted: ", true);
                tele.update();

                //make variables for all errors (for rotate, translate, and strafe)
                double yawError = desiredTag.ftcPose.yaw; // positive error -> robot needs to move right
                double rangeError = desiredTag.ftcPose.range - lateralDistance; // positive error -> robot needs to move forward
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

                //using telemetry to see motor power and the headingError
                tele.addData("Motor power: ",turn);
                tele.addData("error: ", headingError);

                //setting power to all motors
                leftFront.setPower(leftFrontPower);
                rightFront.setPower(rightFrontPower);
                leftRear.setPower(leftBackPower);
                rightRear.setPower(rightBackPower);

            } else {
                //if april tag is not visible, stop motor power
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftRear.setPower(0);
                rightRear.setPower(0);

                //if april tag is not seen through the camera, half power. if power is low, stop
                //if (Math.abs(leftFront.getPower()) < motorPowerCutOff) leftFront.setPower(0);
                //else leftFront.setPower(leftFront.getPower() / 2);

                //if (Math.abs(rightFront.getPower()) < motorPowerCutOff) rightFront.setPower(0);
                //else rightFront.setPower(rightFront.getPower() / 2);

                //if (Math.abs(leftRear.getPower()) < motorPowerCutOff) leftRear.setPower(0);
                //else leftRear.setPower(leftRear.getPower() / 2);

                //if (Math.abs(rightRear.getPower()) < motorPowerCutOff) rightRear.setPower(0);
                //else rightRear.setPower(rightRear.getPower() / 2);

                tele.addData("Tag spotted: ", false);
                tele.addData("Camera on: ", camOn);
                tele.addData("Tag spotted: ", false);
                tele.update();

            }
        }
        waitForStart();
    }
}

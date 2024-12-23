package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MiggyUnLimbetedOuttake implements Subsystem {
    public static double kP = 0.09;
    public double currentPos;
    public double slides_target = 900;
    public Servo outtakeFlipper;
    public DcMotorEx Rlift, Llift;
    boolean waspressedlift = false;
    private Servo outtakeArm1;
    private Servo outtakeArm2;
    private Servo outtakeClaw;

    private boolean fallingEdgeUpCheck, fallingEdgeDownCheck = false;

    @Override
    public void init(HardwareMap hardwareMap) {
        // Initialize servos with corresponding hardware names
        outtakeArm1 = hardwareMap.get(Servo.class, "Servo2");
        outtakeArm2 = hardwareMap.get(Servo.class, "Servo3");
        outtakeClaw = hardwareMap.get(Servo.class, "Servo0");
        outtakeFlipper = hardwareMap.get(Servo.class, "Servo1");
        Rlift = hardwareMap.get(DcMotorEx.class, "rightLift");
        Llift = hardwareMap.get(DcMotorEx.class, "leftLift");

        Rlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Llift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        Rlift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Llift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void reset() {
        Rlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rlift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void Lift(double x) {
        Rlift.setPower(x);
        Llift.setPower(x);
    }


    public void Arm(double t) {
        outtakeArm1.setPosition(t);
        outtakeArm2.setPosition(t);

    }

    @Override
    public void update() {

    }

    @Override
    public void addTelemetry(Telemetry t) {

    }

    public void dontgetstuckonbasket() {
        loiter2();
        Arm(0.5);
    }

    public void loiter1() {
        Arm(0.55);
    }

    public void autoLoiter1() {
        Arm(0.57);
    }

    public void loiter2() {
        // Open the claw (0.44)
        outtakeClaw.setPosition(0.44);
    }

//    public void liftSetPos(Gamepad g) {
//        if(g.right_bumper){
//            waspressedlift = true;
//        }
//        if (!g.right_bumper && waspressedlift){
//            while (true) {
//                Lift(1.0);
//            }
//
//        }
//    }

    public void loiter3() {
        // Set the flipper to position 0.37
        outtakeFlipper.setPosition(0.55);
    }


    public void transfer1() {
        // Move the arm to the initial transfer position (0.30)
        Arm(0.32);

    }


    public void transfer2() {
        // Close the claw (0.63)
        outtakeClaw.setPosition(0.63);
    }

    public void back1() {
        // Move the arm to the extended transfer position (0.8)
        Arm(0.7);
        outtakeFlipper.setPosition(0.15);
    }

    public void backAuton() {
        Arm(0.65);
        outtakeFlipper.setPosition(0.15);
        //close claw
        outtakeClaw.setPosition(0.63);
    }

    public void outtakeFlipper(double pos) {
        outtakeFlipper.setPosition(pos);
    }

    public void back2() {
        // Move the flipper to 0.0 for final transfer

    }

    public void back2Auton() {
        outtakeFlipper.setPosition(0.6);
        Arm(0.22);
    }


    public void score() {
        // Open the claw to release the object (0.44)
        outtakeClaw.setPosition(0.44);
    }

    public void highBasket() {
        PIDLoop(1360);
    }

    public void slidesTransfer() {
        PIDLoop(-100);
    }


    public void autonInit() {
        //flipper shoudl be in loiterng but claw should be lclosed
        autoLoiter1();
        transfer2();
        loiter3();

    }

    public void specimenAutonInit() {
        specimenPickupStart();
        specimenPickupGrab();
    }

    public void SlidesBrake() {
        Rlift.setPower(0);
        Llift.setPower(0);
    }

    public void PIDLoop(double targetPos) {
        double cuurentPos = Rlift.getCurrentPosition();
        double error = targetPos - cuurentPos;

        double out = -(kP * error);

        Rlift.setPower(out);
        Llift.setPower(out);
    }

    public void PIDLoopAuto(double targetPos) {
        double currentPos = -Rlift.getCurrentPosition();
        ElapsedTime elaspedTimer = new ElapsedTime();
        elaspedTimer.startTime();
        while (elaspedTimer.seconds() < 2) {
            double error = targetPos - currentPos;
            double out = -(kP * error);
            Rlift.setPower(out);
            Llift.setPower(out);
            currentPos = -Rlift.getCurrentPosition();
        }
    }

    public double slidesPower() {
        return Rlift.getPower();
    }

    public void specimenPickupStart() {
        Arm(0);
        outtakeFlipper.setPosition(0.45);
    }

    public void specimenPickupGrab() {
        outtakeClaw.setPosition(0.6);
    }

    public void SlidesAllthewaydown(){
        PIDLoop(0);
    }

    public void specimenPickupUp() {
        outtakeFlipper.setPosition(0.7);
    }

    public void specimenSlideUp(Gamepad g, Telemetry telemetry) {
    specimenPickupGrab();

        // falling edge detector
        if (g.dpad_up) {
            fallingEdgeUpCheck = true;
            fallingEdgeDownCheck = false;
        }
        if (g.dpad_down) {
            fallingEdgeUpCheck = false;
            fallingEdgeDownCheck = true;
        }

        if (!g.dpad_up && fallingEdgeUpCheck) {
            slides_target += 50;
            fallingEdgeUpCheck = false;
        } else if (!g.dpad_down && fallingEdgeDownCheck) {
            slides_target -= 50;
            fallingEdgeDownCheck = false;
        }


//        PIDLoop(slides_target);
//        telemetry.addData("Slides target", slides_target);
//        telemetry.addData("Real pos", Rlift.getCurrentPosition());
//        telemetry.update();
        outtakeFlipper.setPosition(0.4);
    }


    public void specimenSlideUp_noadjust() {
        specimenPickupGrab();


        PIDLoop(900);

        outtakeFlipper.setPosition(0.4);
    }
public void specimenScoringPosition() {
        specimenPickupGrab();

        outtakeFlipper.setPosition(0.4);
    }
    public void specimenSlideDown() {
        PIDLoop(600);
    }

    public void specimenRelease() {
        outtakeClaw.setPosition(0.44);
    }


}

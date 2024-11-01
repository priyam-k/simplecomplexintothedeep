package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MiggyUnLimbetedOuttake implements Subsystem {
    private Servo outtakeArm1, outtakeArm2, outtakeClaw, outtakeFlipper;
    public static double kP = 0.006;

    DcMotorEx Rlift,Llift;

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


        Rlift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Llift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void Lift(double x){
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

    public void loiter1() {
        // Set the arm to the latching position at 0.7
        Arm(0.7);
    }

    public void loiter2() {
        // Open the claw (0.44)
        outtakeClaw.setPosition(0.44);
    }

    public void loiter3() {
        // Set the flipper to position 0.37
        outtakeFlipper.setPosition(0.37);
    }

    public void transfer1() {
        // Move the arm to the initial transfer position (0.30)
        Arm(0.30);
    }

    public void transfer2() {
        // Close the claw (0.63)
        outtakeClaw.setPosition(0.63);
    }

    public void back1() {
        // Move the arm to the extended transfer position (0.8)
        Arm(0.7);
    }

    public void back2() {
        // Move the flipper to 0.0 for final transfer

        outtakeFlipper.setPosition(0);
    }

    public void score() {
        // Open the claw to release the object (0.44)
        outtakeClaw.setPosition(0.44);
    }

    public void autonInit() {
        //flipper shoudl be in loiterng but claw should be lclosed
        loiter1();
        transfer2();
        loiter3();
    }
    public void PIDLoop(double targetPos) {
        double cuurentPos = -Rlift.getCurrentPosition();
        double error = targetPos - cuurentPos;

        double out = -(kP * error) ;

        Rlift.setPower(out);
        Llift.setPower(out);

    }
}

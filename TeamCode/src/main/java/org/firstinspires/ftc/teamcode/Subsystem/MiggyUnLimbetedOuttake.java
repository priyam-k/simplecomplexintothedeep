package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MiggyUnLimbetedOuttake implements Subsystem {
    public double currentPos;
    private Servo outtakeArm1, outtakeArm2, outtakeClaw, outtakeFlipper;
    public static double kP = 0.006;
    boolean waspressedlift = false;

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

        //TODO: EXPIRAMENT WITH REVERSING ONE MOTOR
        // Rlift.setDirection(DcMotorSimple.Direction.REVERSE);

        Rlift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        // Set arm to the latching position and reset claw and flipper
        Arm(0.7);
        outtakeClaw.setPosition(0.44);
        outtakeFlipper.setPosition(0.37);
    }

    public void loiter2() {
        // Re-open claw if it was changed
        outtakeClaw.setPosition(0.44);
        // Confirm flipper is in position
        outtakeFlipper.setPosition(0.37);
        Arm(0.7); // Ensure arm is still in latching position
    }

    public void loiter3() {
        // Confirm flipper position
        outtakeFlipper.setPosition(0.37);
        outtakeClaw.setPosition(0.44); // Re-confirm claw position
        Arm(0.7); // Confirm arm position again
    }

    public void transfer1() {
        // Prepare arm for transfer, reset claw and flipper if necessary
        Arm(0.30);
        outtakeClaw.setPosition(0.44); // Re-open claw to prevent accidental drops
        outtakeFlipper.setPosition(0.37); // Ensure flipper is reset
    }

    public void transfer2() {
        // Close claw for holding during transfer
        outtakeClaw.setPosition(0.63);
        Arm(0.30); // Ensure arm stays in initial transfer position
        outtakeFlipper.setPosition(0.37); // Confirm flipper position
    }

    public void back1() {
        // Extend arm further and reset claw and flipper
        Arm(0.65);
        outtakeClaw.setPosition(0.63); // Keep claw closed during transfer
        outtakeFlipper.setPosition(0.37); // Confirm flipper position
    }

    public void back2() {
        // Move flipper for final transfer position
        Arm(0.65); // Ensure arm position remains extended
        outtakeClaw.setPosition(0.63); // Ensure claw stays closed
        outtakeFlipper.setPosition(0.0); // Move flipper to final transfer
    }

    public void score() {
        // Open claw to release object
        outtakeClaw.setPosition(0.44); // Open claw to release
        Arm(0.65); // Keep arm in scoring position
        outtakeFlipper.setPosition(0.0); // Confirm flipper in scoring position
    }


    public void autonInit() {
        //flipper shoudl be in loiterng but claw should be lclosed
        loiter1();
        transfer2();
        loiter3();
    }

    public void SlidesBrake(){
        Rlift.setPower(0);
        Llift.setPower(0);
    }
    public void PIDLoop(double targetPos) {
        double cuurentPos = Rlift.getCurrentPosition();
            double error = targetPos - cuurentPos;

            double out = -(kP * error) ;

            Rlift.setPower(out);
            Llift.setPower(out);

    }
    public void PIDLoopAuto(double targetPos) {
        ElapsedTime timer = new ElapsedTime();
        currentPos = -Rlift.getCurrentPosition();
        while( currentPos < targetPos-300) {
            double error = targetPos - currentPos;
            double out = -(kP * error);
            Rlift.setPower(out);
            Llift.setPower(out);
            currentPos = -Rlift.getCurrentPosition();
        }
    }
}

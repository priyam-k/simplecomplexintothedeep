package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MiggyUnLimbetedOuttake implements Subsystem {
    private Servo outtakeArm1, outtakeArm2, outtakeClaw, outtakeFlipper;

    @Override
    public void init(HardwareMap hardwareMap) {
        // Initialize servos with corresponding hardware names
        outtakeArm1 = hardwareMap.get(Servo.class, "Servo2");
        outtakeArm2 = hardwareMap.get(Servo.class, "Servo3");
        outtakeClaw = hardwareMap.get(Servo.class, "Servo0");
        outtakeFlipper = hardwareMap.get(Servo.class, "Servo1");
    }


    public void Arm(double t) {
        outtakeArm1.setPosition(t);
        outtakeArm2.setPosition(t);

    }

    @Override
    public void update() {

    }

    @Override
    public void addTelemtry(Telemetry t) {

    }

    // Outtake claw open 0.44, outtake claw close 0.63
    public void Loiter() {
        // Set the arm to the latching position at 0.6
        Arm(0.6);

        // Open the claw (0.44)
        outtakeClaw.setPosition(0.44);

        // Set the flipper to position 0.4
        outtakeFlipper.setPosition(0.4);
    }

    public void Transfer() {
        // Move the arm to the initial transfer position (0.28)
       Arm(0.28);

        // Close the claw (0.63)
        outtakeClaw.setPosition(0.63);

    }

    public void Back() {

        // Wait for confirmation from intake claw: vector
        // After confirmation, move the arm to the extended transfer position (0.8)
        Arm(0.8);

        // Move the flipper to 0.05 for the final transfer
        outtakeFlipper.setPosition(0.05);


    }

    public void Score() {
        // Open the claw to release the object (0.44)
        outtakeClaw.setPosition(0.44);
    }
}

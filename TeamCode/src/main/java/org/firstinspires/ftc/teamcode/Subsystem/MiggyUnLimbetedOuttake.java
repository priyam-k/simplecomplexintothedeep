package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MiggyUnLimbetedOuttake implements Subsystem {
    private Servo outtakeArm, outtakeClaw, outtakeFlipper;

    @Override
    public void init(HardwareMap hardwareMap) {
        // Initialize servos with corresponding hardware names
        outtakeArm = hardwareMap.get(Servo.class, "outtakeArm");
        outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");
        outtakeFlipper = hardwareMap.get(Servo.class, "outtakeFlipper");
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
        outtakeArm.setPosition(0.6);

        // Open the claw (0.44)
        outtakeClaw.setPosition(0.44);

        // Set the flipper to position 0.4
        outtakeFlipper.setPosition(0.4);
    }

    public boolean Transfer(Enum state) {
        // Move the arm to the initial transfer position (0.28)
        outtakeArm.setPosition(0.28);

        // Close the claw (0.63)
        outtakeClaw.setPosition(0.63);

        // Wait for confirmation from intake claw: vector
        if (state == StateMachines.Intake.LOITER) {
            // After confirmation, move the arm to the extended transfer position (0.8)
            outtakeArm.setPosition(0.8);

            // Move the flipper to 0.05 for the final transfer
            outtakeFlipper.setPosition(0.05);

            return true;
        }
        return false;

    }

    public void Score() {
        // Open the claw to release the object (0.44)
        outtakeClaw.setPosition(0.44);
    }
}

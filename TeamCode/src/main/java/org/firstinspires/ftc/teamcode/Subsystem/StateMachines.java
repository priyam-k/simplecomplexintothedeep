package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.sfdev.assembly.state.State;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

public class StateMachines {

    public static StateMachine getIntakeStateMachine(EnableHand hand, Gamepad gamepad) {
        return new StateMachineBuilder()
                .state(Intake.SCANNING)
                .onEnter(hand::scan)
                .transition(() -> gamepad.b)


                .state(Intake.HOVERING)
                .onEnter(hand::hover)
                .transition(() -> gamepad.a)

                .state(Intake.PICKUP)
                .onEnter(hand::pickup)
                .transition(() -> gamepad.a)

                .state(Intake.TRANSFER)
                .onEnter(hand::transfer)
                .transition(() -> gamepad.a, Intake.SCANNING)

                .build();
    }
    public static StateMachine getOuttakeStateMachine(Outtake out, Gamepad gamepad) {
        return new StateMachineBuilder()
                .state(Transfer.IDLING)
                .onEnter(out::idle)
                .transition(() -> gamepad.a)


                .state(Transfer.GRABBING)
                .onEnter(out::grab)
                .transition(() -> gamepad.a)

                .state(Transfer.FLIPPING)
                .onEnter(out::flip)
                .transition(() -> gamepad.a)

                .state(Transfer.DROPPING)
                .onEnter(out::drop)
                .transition(() -> gamepad.a, Transfer.IDLING)

                .build();
    }

    enum Transfer {
        IDLING, GRABBING, FLIPPING, DROPPING
    }

    enum Intake {
        SCANNING, HOVERING, PICKUP, TRANSFER
    }
}

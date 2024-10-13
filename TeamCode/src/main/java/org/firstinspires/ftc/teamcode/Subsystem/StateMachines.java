package org.firstinspires.ftc.teamcode.Subsystem;

import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class StateMachines {
    enum Transfer {
        IDLING, GRABBING, FLIPPING, DROPPING
    }
    public static StateMachine getOuttakeStateMachine(Outtake out) {
        return new StateMachineBuilder()
                .state(Transfer.IDLING)
                .onEnter(out::idle)
                .transitionTimed(0.2) // 200 millis non-blocking "sleep"

                .state(Transfer.GRABBING)
                .onEnter(out::grab)
                .transitionTimed(0.2)

                .state(Transfer.FLIPPING)
                .onEnter(out::flip)
                .transitionTimed(0.2)

                .state(Transfer.DROPPING)
                .onEnter(out::drop)
                .transitionTimed(0.2, Transfer.IDLING)

                .build();
    }
}

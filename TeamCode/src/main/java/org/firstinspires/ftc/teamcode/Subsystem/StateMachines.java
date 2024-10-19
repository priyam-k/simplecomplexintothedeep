package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

public class StateMachines {

    public static StateMachine getIntakeStateMachine(EnableHand hand, Gamepad gamepad) {
        return new StateMachineBuilder()
                .state(Intake.SCANNING)
                .onEnter(hand::scan)
                .transition(() -> gamepad.a, Intake.HOVERING)

                .state(Intake.HOVERING)
                .onEnter(hand::hover)
                .transition(() -> gamepad.a, Intake.PICKUP)

                .state(Intake.PICKUP)
                .onEnter(hand::pickup)
                .transition(() -> gamepad.a, Intake.TRANSFER)

                .state(Intake.TRANSFER)
                .onEnter(hand::transfer)
                .transition(() -> gamepad.a, Intake.LOITER)

                .state(Intake.LOITER)
                .onEnter(hand::loiter)
                .transition(() -> gamepad.a, Intake.SCANNING)

                .build();
    }

    public static StateMachine getOuttakeStateMachine(MiggyUnLimbetedOuttake out, Gamepad gamepad, StateMachine intake) {
        return new StateMachineBuilder()
                .state(Outtake.LOITERING)
                .onEnter(out::Loiter)
                .transition(() -> gamepad.b, Outtake.TRANSFERRING)

                .state(Outtake.TRANSFERRING)
                .onEnter(out::Transfer)
                .transition(()->intake.getState() == Intake.LOITER)

                .state(Outtake.BACK)
                .onEnter(out::Back)
                .transition(()->gamepad.b, Outtake.SCORING)

                .state(Outtake.SCORING)
                .onEnter(out::Score)
                .transition(() -> gamepad.b, Outtake.LOITERING)

                .build();
    }

    enum Outtake {
        LOITERING, TRANSFERRING, SCORING, BACK
    }

    enum Intake {
        SCANNING, HOVERING, PICKUP, TRANSFER, LOITER
    }
}

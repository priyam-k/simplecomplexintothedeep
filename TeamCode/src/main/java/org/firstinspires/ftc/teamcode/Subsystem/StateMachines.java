package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import java.util.concurrent.atomic.AtomicBoolean;

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
                .transition(() -> gamepad.a, Intake.LOITER)

                .state(Intake.LOITER)
                .onEnter(hand::loiter)
                .transition(() -> gamepad.a, Intake.SCANNING)

                .build();
    }

    public static StateMachine getOuttakeStateMachine(MiggyUnLimbetedOuttake out, Gamepad gamepad, StateMachine intake) {
        AtomicBoolean check = new AtomicBoolean(false);
        return new StateMachineBuilder()
                .state(Outtake.LOITERING)
                .onEnter(out::Loiter)
                .transition(() -> gamepad.a, Outtake.TRANSFERRING)

                .state(Outtake.TRANSFERRING)
                .onEnter(() -> {
                    check.set(out.Transfer(intake.getState()));
                })
                .transition(() -> check.get() && gamepad.a, Outtake.SCORING)
                .onEnter(()->check.set(false))

                .state(Outtake.SCORING)
                .onEnter(out::Score)
                .transition(() -> gamepad.a, Outtake.LOITERING)

                .build();
    }

    enum Outtake {
        LOITERING, TRANSFERRING, SCORING
    }

    public enum Intake {
        SCANNING, HOVERING, PICKUP, TRANSFER, LOITER
    }
}

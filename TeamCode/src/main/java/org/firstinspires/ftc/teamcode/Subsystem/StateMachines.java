package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

public class StateMachines {
    public static StateMachine getIntakeStateMachine(EnableHand hand, Gamepad gamepad) {
        return new StateMachineBuilder()
                .state(Intake.LOITER)
                .onEnter(hand::loiter)
                .transition(() -> gamepad.right_bumper, Intake.SCANNING1)

                .state(Intake.SCANNING1)
                .onEnter(hand::scan1)
                .transitionTimed(0.25, Intake.SCANNING2)

                .state(Intake.SCANNING2)
                .onEnter(hand::scan2)
                .transitionTimed(0.25, Intake.SCANNING3)

                .state(Intake.SCANNING3)
                .onEnter(hand::scan3)
                .transitionTimed(0.25, Intake.SCANNING4)

                .state(Intake.SCANNING4)
                .onEnter(hand::scan4)
                .transition(() -> gamepad.right_bumper, Intake.WAIT)

                .state(Intake.WAIT)
                .transitionTimed(0.25, Intake.HOVERING)

                .state(Intake.HOVERING)
                .onEnter(hand::hover1)
                .loop(hand::hover2)
                .transition(() -> gamepad.right_bumper, Intake.PICKUP1)

                .state(Intake.PICKUP1)
                .onEnter(hand::pickup1)
                .transitionTimed(0.25, Intake.PICKUP2)

                .state(Intake.PICKUP2)
                .onEnter(hand::pickup2)
                .transition(() -> gamepad.right_bumper, Intake.TRANSFER1)

                .waitState(0.5)

                .state(Intake.TRANSFER1)
                .onEnter(hand::transfer1point5)
                .transitionTimed(0.25, Intake.TRANSFER2)

                .state(Intake.TRANSFER2)
                .onEnter(hand::transfer2)
                .transition(() -> gamepad.right_bumper, Intake.WAIT2)

                .state(Intake.WAIT2)
                .transitionTimed(0.25, Intake.LOITER)

                .build();
    }

    public enum Intake {
        SCANNING1, SCANNING2, SCANNING3, SCANNING4,
        WAIT,WAIT2,
        HOVERING,
        PICKUP1, PICKUP2,
        TRANSFER1, TRANSFER2, TRANSFER3,
        LOITER
    }

    public static StateMachine getOuttakeStateMachine(MiggyUnLimbetedOuttake out, Gamepad gamepad, StateMachine intake) {
        return new StateMachineBuilder()
                .state(Outtake.LOITERING1)
                .onEnter(out::loiter1)
                .transitionTimed(0.25, Outtake.LOITERING2)

                .state(Outtake.LOITERING2)
                .onEnter(out::loiter2)
                .transitionTimed(0.25, Outtake.LOITERING3)

                .state(Outtake.LOITERING3)
                .onEnter(out::loiter3)
                .transition(() -> gamepad.left_bumper  && intake.getState() == Intake.TRANSFER2, Outtake.TRANSFERRING1)

                .state(Outtake.TRANSFERRING1)
                .onEnter(out::transfer1)
                .transitionTimed(0.75 , Outtake.TRANSFERRING2)

                .state(Outtake.TRANSFERRING2)
                .onEnter(out::transfer2)
                .transition(()->intake.getState() == Intake.LOITER, Outtake.BACK1)

                .state(Outtake.BACK1)
                .onEnter(out::back1)
                .transitionTimed(0.5, Outtake.BACK2)

                .state(Outtake.BACK2)
                .onEnter(out::back2)
                .transition(() -> gamepad.left_bumper, Outtake.SCORING)

                .waitState(0.5)

                .state(Outtake.SCORING)
                .onEnter(out::score)
                .transition(() -> gamepad.left_bumper, Outtake.LOITERING1)

                .build();
    }

    public enum Outtake {
        LOITERING1, LOITERING2, LOITERING3,
        TRANSFERRING1, TRANSFERRING2,
        BACK1, BACK2,
        SCORING
    }



}

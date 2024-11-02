package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

public class StateMachines {
    public static StateMachine getMachine(MiggyUnLimbetedOuttake out, EnableHand hand, Gamepad gamepad) {
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
                .transition(() -> gamepad.left_bumper, Intake.LOITER)


                .state(Intake.WAIT)
                .transitionTimed(0.25, Intake.HOVERING)

                .state(Intake.HOVERING)
                .onEnter(hand::hover1)
                .loop(hand::hover2)
                .transition(() -> gamepad.right_bumper, Intake.PICKUP1)
                .transition(() -> gamepad.left_bumper, Intake.SCANNING4)


                .state(Intake.PICKUP1)
                .onEnter(hand::pickup1)
                .transitionTimed(0.25, Intake.PICKUP2)

                .state(Intake.PICKUP2)
                .onEnter(hand::pickup2)
                .transition(() -> gamepad.right_bumper, Intake.TRANSFER1)
                .transition(() -> gamepad.left_bumper, Intake.HOVERING)


                .waitState(0.5)

                .state(Intake.TRANSFER1)
                .onEnter(hand::transfer1)
                .transitionTimed(0.25, Intake.TRANSFER2)

                .waitState(0.5)

                .state(Intake.TRANSFER2)
                .onEnter(hand::transfer1point5)
                .transitionTimed(0.25, Intake.TRANSFER3)

                .state(Intake.TRANSFER3)
                .onEnter(hand::transfer2)
                .transition(() -> gamepad.right_bumper, Intake.WAIT2)
                .transition(() -> gamepad.left_bumper, Intake.PICKUP2)


                .state(Intake.WAIT2)
                .transitionTimed(0.25, Outtake.LOITERING1)

                .state(Outtake.LOITERING1)
                .onEnter(out::loiter1)
                .transitionTimed(0.25, Outtake.LOITERING2)

                .state(Outtake.LOITERING2)
                .onEnter(out::loiter2)
                .transitionTimed(0.25, Outtake.LOITERING3)

                .state(Outtake.LOITERING3)
                .onEnter(out::loiter3)
                .transition(() -> gamepad.right_bumper, Outtake.TRANSFERRING1)
                .transition(() -> gamepad.left_bumper, Outtake.LOITERING3)

                .state(Outtake.TRANSFERRING1)
                .onEnter(out::transfer1)
                .transitionTimed(0.75, Outtake.TRANSFERRING2)

                .state(Outtake.TRANSFERRING2)
                .onEnter(out::transfer2)
                .transitionTimed(0.25)

                .state(Outtake.BACK1)
                .onEnter(out::back1)
                .transitionTimed(0.5, Outtake.BACK2)

                .state(Outtake.BACK2)
                .onEnter(out::back2)
                .transition(() -> gamepad.right_bumper, Outtake.SCORING)
                .transition(() -> gamepad.left_bumper, Outtake.TRANSFERRING2)

                .waitState(0.5)

                .state(Outtake.SCORING)
                .onEnter(out::score)
                .transition(() -> gamepad.right_bumper, Outtake.LOITERING1)
                .transition(() -> gamepad.left_bumper, Outtake.BACK2)


                .build();
    }

    public enum Intake {
        SCANNING1, SCANNING2, SCANNING3, SCANNING4,
        WAIT, WAIT2, // WAIT2 is used at the end of the Intake Transfer Seq. as a buffer state between transfer and loiter
        HOVERING,
        PICKUP1, PICKUP2,
        TRANSFER1, TRANSFER2, TRANSFER3,
        LOITER
    }

    public enum Outtake {
        LOITERING1, LOITERING2, LOITERING3,
        TRANSFERRING1, TRANSFERRING2,
        BACK1, BACK2,
        SCORING
    }


}

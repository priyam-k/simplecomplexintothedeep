package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

public class StateMachines {
    public static StateMachine getIntakeStateMachine(EnableHand hand, Gamepad gamepad) {
        return new StateMachineBuilder()
                .state(Intake.LOITER)
                .onEnter(hand::loiter)
                .transition(() -> gamepad.a, Intake.SCANNING1)

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
                .transition(() -> gamepad.a, Intake.WAIT)

                .state(Intake.WAIT)
                .transitionTimed(0.25, Intake.HOVERING)

                .state(Intake.HOVERING)
                .onEnter(hand::hover1)
                .loop(hand::hover2)
                .transition(() -> gamepad.a, Intake.PICKUP1)

                .state(Intake.PICKUP1)
                .onEnter(hand::pickup1)
                .transitionTimed(0.25, Intake.PICKUP2)

                .state(Intake.PICKUP2)
                .onEnter(hand::pickup2)
                .transition(() -> gamepad.a, Intake.TRANSFER1)


                .state(Intake.TRANSFER1)
                .onEnter(hand::transfer1point5)
                .transitionTimed(0.25, Intake.TRANSFER2)

                .state(Intake.TRANSFER2)
                .onEnter(hand::transfer2)
                .transition(() -> gamepad.a, Intake.WAIT2)

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
                .transitionTimed(0.25,Outtake.LOITERING2)

                .state(Outtake.LOITERING2)
                .onEnter(out::loiter2)
                .transitionTimed(0.05, Outtake.SLIDESDOWN)

                .state(Outtake.SLIDESDOWN)
                .onEnter(out::slidesTransfer)
                .loop(out::slidesTransfer)
                .transitionTimed(1, Outtake.LOITERING3)

                .state(Outtake.LOITERING3)
                .onEnter(() -> {
                    out.loiter3();
                    out.SlidesBrake();
                })
                .transition(() -> gamepad.b  && intake.getState() == Intake.TRANSFER2, Outtake.TRANSFERRING1)
                .transition(() -> gamepad.x && intake.getState() == Intake.TRANSFER2, Outtake.TRANSFERRING1OBSZONE)
                .transition(() -> gamepad.triangle && (intake.getState() == Intake.SCANNING4), Outtake.WAIT1) //GAMEPAD TRIANGLE MEANS y


                .state(Outtake.WAIT1)
                .transitionTimed(0.25,Outtake.SPECIMENPICKUPSTART)

                .state(Outtake.SPECIMENPICKUPSTART)
                .onEnter(out::specimenPickupStart)
                .transition(() -> gamepad.triangle, Outtake.WAIT2)

                .state(Outtake.WAIT2)
                .transitionTimed(0.25,Outtake.SPECIMENPICKUPGRAB)

                .state(Outtake.SPECIMENPICKUPGRAB)
                .onEnter(out::specimenPickupGrab)
                .transition(()->gamepad.triangle, Outtake.WAIT3)

                .state(Outtake.WAIT3)
                .transitionTimed(0.25,Outtake.SPECIMENPICKUPUP)


                .state(Outtake.SPECIMENPICKUPUP)
                .onEnter(out::specimenPickupUp)
                .transition(() -> gamepad.triangle, Outtake.WAIT4)

                .state(Outtake.WAIT4)
                .transitionTimed(0.25, Outtake.SPECIMENSLIDEUP)

                .state(Outtake.SPECIMENSLIDEUP)
                .onEnter(() -> out.specimenSlideUp(gamepad))
                .loop(() -> out.specimenSlideUp(gamepad))
                .transition(() -> gamepad.triangle, Outtake.WAIT5)

                .state(Outtake.WAIT5)
                .transitionTimed(0.25,Outtake.SPECIMENSLIDEDOWN)


                .state(Outtake.SPECIMENSLIDEDOWN)
                .onEnter(out::specimenSlideDown)
                .transitionTimed(0.5, Outtake.SPECIMENRELEASE)

                .state(Outtake.SPECIMENRELEASE)
                .onEnter(out::specimenRelease)
                .transition(() -> gamepad.y, Outtake.LOITERING1)

                .waitState(0.5)

                .state(Outtake.TRANSFERRING1)
                .onEnter(out::transfer1)
                .transitionTimed(0.25 , Outtake.TRANSFERRING2)

                .state(Outtake.TRANSFERRING2)
                .onEnter(out::transfer2)
                .transition(()->intake.getState() == Intake.LOITER, Outtake.BACK1HIGH)

                .state(Outtake.TRANSFERRING1OBSZONE)
                .onEnter(out::transfer1)
                .transitionTimed(0.25 , Outtake.TRANSFERRING2OBSZONE)

                .state(Outtake.TRANSFERRING2OBSZONE)
                .onEnter(out::transfer2)
                .transition(()->intake.getState() == Intake.LOITER, Outtake.BACK1OBSZON)

                .state(Outtake.BACK1HIGH)
                  .onEnter(out::back1)
                //.loop(out::highBasket)
                .transition(()->gamepad.b, Outtake.SCORING)

                .state(Outtake.BACK1OBSZON)
                .onEnter(out::back1)
                .transition(()->gamepad.b, Outtake.SCORING)

                .state(Outtake.SCORING)
                .onEnter(out::score)
                .transitionTimed(0.75, Outtake.LOITERING1)

                .build();
    }

    public enum Outtake {
        LOITERING1, LOITERING2, LOITERING3,
        TRANSFERRING1, TRANSFERRING2,
        BACK1HIGH, BACK2,BACK1OBSZON,TRANSFERRING1OBSZONE,TRANSFERRING2OBSZONE,
        SCORING,
        SLIDESDOWN,
        SPECIMENPICKUPSTART, SPECIMENPICKUPGRAB, SPECIMENPICKUPUP, SPECIMENSLIDEUP, SPECIMENSLIDEDOWN, BRAKE2, SPECIMENRELEASE,
        WAIT1,WAIT2,WAIT3,WAIT4,WAIT5
    }


}
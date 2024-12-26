package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class StateMachineV2 {
    public static StateMachine getIntakeStateMachine(EnableHand hand, Gamepad gamepad) {
        return new StateMachineBuilder().state(Intake.LOITER).onEnter(hand::loiter).transition(() -> gamepad.a, Intake.SCANNING1)

                .state(Intake.SCANNING1).onEnter(hand::scan1).transitionTimed(0.25, Intake.SCANNING2)

                .state(Intake.SCANNING2).onEnter(hand::scan2).transitionTimed(0.25, Intake.SCANNING3)

                .state(Intake.SCANNING3).onEnter(hand::scan3).transitionTimed(0.25, Intake.SCANNING4)

                .state(Intake.SCANNING4).onEnter(hand::scan4).transition(() -> gamepad.a, Intake.WAIT)

                .state(Intake.WAIT).transitionTimed(0.25, Intake.HOVERING)

                .state(Intake.HOVERING).onEnter(hand::hover1).loop(hand::hover2).transition(() -> gamepad.a, Intake.PICKUP1)

                .state(Intake.PICKUP1).onEnter(hand::pickup1).transitionTimed(0.25, Intake.PICKUP2)

                .state(Intake.PICKUP2).onEnter(hand::pickup2).transition(() -> gamepad.a, Intake.TRANSFER1)


                .state(Intake.TRANSFER1).onEnter(hand::transfer1point5).transitionTimed(0.25, Intake.TRANSFER2)

                .state(Intake.TRANSFER2).onEnter(hand::transfer2).transition(() -> gamepad.a, Intake.WAIT2)

                .state(Intake.WAIT2).transitionTimed(0.25, Intake.LOITER)

                .build();
    }

    public static StateMachine getSpecimenStateMachine(EnableHand hand, MiggyUnLimbetedOuttake out, Gamepad gamepad, Telemetry telemetry) {

        return new StateMachineBuilder()

                .state(StateMachineV2.Outtake.SPECIMENPICKUPSTART)
                .onEnter(() -> {
                    out.specimenPickupStart();
                    hand.setSwingArmAngle(90);
                    out.loiter2();
                    out.SlidesBrake();
                })
                .transition(() -> gamepad.a, StateMachineV2.Outtake.WAIT2)

                .state(StateMachineV2.Outtake.WAIT2)
                .transitionTimed(0.15, StateMachineV2.Outtake.SPECIMENPICKUPGRAB)

                .state(StateMachineV2.Outtake.SPECIMENPICKUPGRAB)
                .onEnter(out::specimenPickupGrab)
                .transitionTimed(0.15, StateMachineV2.Outtake.WAIT3)

                .state(StateMachineV2.Outtake.WAIT3)
                .onEnter(() -> gamepad.rumble(500))
                .transitionTimed(0.15, StateMachineV2.Outtake.SPECIMENPICKUPUP)

                .state(StateMachineV2.Outtake.SPECIMENPICKUPUP)
                .onEnter(out::specimenPickupUp)
                .transition(() -> gamepad.a, StateMachineV2.Outtake.WAIT4)
                .transition(() -> gamepad.b, Outtake.SPECIMENPICKUPSTART)// escape state

                .state(StateMachineV2.Outtake.WAIT4)
                .transitionTimed(0.15, Outtake.SPECIMENSLIDEUP)
                //manul lift

                .state(StateMachineV2.Outtake.SPECIMENSLIDEUP)
//                .loop(() -> out.specimenSlideUp(gamepad, telemetry)
                .onEnter(() -> out.outtakeFlipper.setPosition(0.4))
                .transition(() -> gamepad.a, Outtake.WAIT5)
//
                .state(StateMachineV2.Outtake.WAIT5)
                .transitionTimed(0.5, Outtake.SPECIMENRELEASE)


//                .state(StateMachineV2.Outtake.SPECIMENSLIDEDOWN)
//                .onEnter(out::specimenSlideDown)
//                .transitionTimed(0.25, StateMachineV2.Outtake.SPECIMENRELEASE)

                .state(StateMachineV2.Outtake.SPECIMENRELEASE)
                .onEnter(out::specimenRelease)
                .transition(() -> gamepad.a, Outtake.SPECIMENPICKUPSTART).build();

    }

    public static StateMachine getOuttakeStateMachine(MiggyUnLimbetedOuttake out, Gamepad gamepad, StateMachine intake) {
        return new StateMachineBuilder().state(Outtake.LOITERING1).onEnter(out::loiter1).transitionTimed(0.25, Outtake.LOITERING2)

                .state(Outtake.LOITERING2).onEnter(out::loiter2).transitionTimed(0.05, Outtake.LOITERING3)

//                .state(Outtake.SLIDESDOWN).onEnter(out::slidesTransfer).loop(out::slidesTransfer).transitionTimed(1, Outtake.LOITERING3)

                .state(Outtake.LOITERING3).onEnter(() -> {
                    out.loiter3();
                    out.SlidesBrake();
                }).transition(() -> gamepad.b && intake.getState() == Intake.TRANSFER2, Outtake.TRANSFERRING1) //GAMEPAD TRIANGLE MEANS y


                .waitState(0.5)

                .state(Outtake.TRANSFERRING1)
                .onEnter(out::transfer1).transitionTimed(0.25, Outtake.TRANSFERRING2)

                .state(Outtake.TRANSFERRING2)
                .onEnter(out::transfer2).transition(() -> intake.getState() == Intake.LOITER, Outtake.BACK1HIGH)


                .state(Outtake.BACK1HIGH).onEnter(out::back1)
                //.loop(out::highBasket)
                .transition(() -> gamepad.b, Outtake.SCORING)


                .state(Outtake.SCORING).onEnter(out::score).transitionTimed(0.75, Outtake.LOITERING1)

                .build();
    }

    public enum Intake {
        SCANNING1, SCANNING2, SCANNING3, SCANNING4, WAIT, WAIT2, HOVERING, PICKUP1, PICKUP2, TRANSFER1, TRANSFER2, TRANSFER3, LOITER
    }

    public enum Outtake {
        LOITERING1, LOITERING2, LOITERING3, TRANSFERRING1, TRANSFERRING2, BACK1HIGH, BACK2, BACK1OBSZON, TRANSFERRING1OBSZONE, TRANSFERRING2OBSZONE, SCORING, SLIDESDOWN, SPECIMENPICKUPSTART, SPECIMENPICKUPGRAB, SPECIMENPICKUPUP, SPECIMENSLIDEUP, SPECIMENSLIDEDOWN, BRAKE2, SPECIMENRELEASE, WAIT1, WAIT2, WAIT3, WAIT4, WAIT5
    }


}
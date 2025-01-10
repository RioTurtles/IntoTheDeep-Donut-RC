package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp v2.0")
public class TeleoperatedV2 extends LinearOpMode {
    boolean reverting, initialExtension, retracting;
    boolean quickTransferQueued, fullTransferQueued;
    boolean specimenScored;
    double current;
    Double target;

    @Override
    public void runOpMode() {
        Project3Hardware robot = new Project3Hardware(hardwareMap);
        State state = State.INITIALISED;
        Gamepad gamepad = new Gamepad();
        Gamepad lastGamepad = new Gamepad();
        Gamepad operator = new Gamepad();
        Gamepad lastOperator = new Gamepad();
        ElapsedTime timer1 = new ElapsedTime();
        ElapsedTime timer2 = new ElapsedTime();
        ElapsedTime timer3 = new ElapsedTime();

        reverting = false;
        initialExtension = false;
        retracting = false;
        quickTransferQueued = false;
        fullTransferQueued = false;
        specimenScored = false;

        robot.scoringMode = "Basket";
        robot.scoringHeight = "High";

        waitForStart();

        while (opModeIsActive()) {
            lastGamepad.copy(gamepad);
            lastOperator.copy(operator);
            gamepad.copy(gamepad1);
            operator.copy(gamepad2);
            double vertical = -gamepad.left_stick_y;
            double horizontal = gamepad.left_stick_x;
            double pivot = gamepad.right_stick_x;
            double heading = robot.getIMUYaw();

            IntakeControls intakeControls = () -> {
                if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                    // Toggle intake on/off
                    if (robot.intakeReversed) robot.intakeOn();
                    else {
                        if (robot.intakeOn) robot.intakeOff();
                        else robot.intakeOn();
                    }
                }

                if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                    // Toggle intake up/down
                    if (robot.intakeUp) {robot.intake.setIntake(); robot.intakeOn();}
                    else {robot.intake.setTransfer(); robot.intakeOff();}
                }

                if (gamepad.circle && !lastGamepad.circle) {
                    // Toggle intake direction (inwards/outwards)
                    if (robot.intakeReversed) robot.intakeOn();
                    else robot.intakeReverse();
                }

                if (gamepad.left_trigger > 0 && lastGamepad.left_trigger > 0) {
                    robot.linearExtension.setPositionPreset(true);
                    robot.intake.setExtended();
                    robot.intakeOn();
                } else if (!(gamepad.left_trigger > 0) && lastGamepad.left_trigger > 0) {
                    retracting = true;
                    timer3.reset();
                    robot.intakeOff();
                    robot.linearExtension.setPositionPreset(false);
                }

                if (gamepad.right_trigger > 0 && lastGamepad.right_trigger > 0) {
                    if (initialExtension) {
                        timer3.reset();
                        initialExtension = false;
                    }

                    robot.linearExtension.setPositionPreset(true);
                    robot.intakeOn();
                    if (timer3.milliseconds() > 400) robot.intake.setExtended();
                } else if (!(gamepad.right_trigger > 0) && lastGamepad.right_trigger > 0) {
                    robot.intake.setTransfer();
                    robot.intakeOff();
                    robot.linearExtension.setPositionPreset(false);
                    initialExtension = true;
                }

                if (retracting && timer3.milliseconds() > 500) {
                    robot.intake.setTransfer();
                    retracting = false;
                }
            };

            if (state == State.INITIALISED) {
                robot.bucket.setPositionPreset("Transfer");
                robot.intake.setTransfer();
                robot.linearExtension.setPositionPreset(false);

                if (gamepad.right_bumper) {
                    robot.intake.pwmEnable();
                    robot.intake.setIntake();
                    robot.intakeOn();
                    state = State.INTAKE_READY;
                    timer1.reset();
                    continue;
                }
            }

            if (state == State.INTAKE_READY) {
                if (robot.scoringMode.equals("Basket")) {
                    robot.sliders.setPower(0);
                    robot.bucket.setPositionPreset("Transfer");
                    intakeControls.call();

                    if (gamepad.cross && !lastGamepad.cross) {
                        // Full transfer (next state)
                        robot.intakeSlow();
                        state = State.TRANSFER_TRANSITION;
                        timer1.reset();
                    }

                    if (gamepad.triangle && !lastGamepad.triangle) {
                        // Quick transfer (next state)
                        state = State.TRANSFERRING;
                        timer1.reset();
                    }
                } else {
                    if (timer1.milliseconds() > 1000) robot.intake.pwmDisable();

                    robot.linearExtension.setPositionPreset(false);
                    robot.intakeOff();
                    robot.retractSlider();

                    if (gamepad.right_bumper) {
                        state = State.TRANSITION_TO_SCORING_CHAMBER;
                        timer1.reset();
                    }
                }

                if (gamepad.share) {
                    robot.retractSlider();
                    robot.intakeOn();
                    robot.intake.setIntake();
                }

                if (gamepad.options) {
                    robot.retractSlider();
                    robot.intakeOff();
                    robot.intake.setTransfer();
                    timer1.reset();
                }
            }

            if (state == State.TRANSFER_TRANSITION) {
                robot.bucket.setPositionPreset("Transfer");
                robot.intake.setTransfer();
                robot.intakeOff();
                if (timer1.milliseconds() > 700) {state = State.TRANSFERRING; timer1.reset();}
                if (gamepad.triangle) {state = State.TRANSFERRING; timer1.reset();}

                if (gamepad.left_bumper) {
                    robot.intake.pwmEnable();
                    robot.intake.setIntake();
                    robot.intakeOn();
                    state = State.INTAKE_READY;
                    continue;
                }
            }

            if (state == State.TRANSFERRING) {
                if (timer1.milliseconds() > 450 || reverting) {
                    if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                        reverting = false;
                        robot.bucket.setPositionPreset("Lift");
                        state = State.TRANSITION_TO_SCORING_BASKET;
                    }
                    robot.intakeOff();
                    robot.intake.setRaised();
                }
                else robot.intakeReverse();

                if (gamepad.left_bumper) {
                    robot.intake.pwmEnable();
                    robot.intake.setIntake();
                    robot.intakeOn();
                    state = State.INTAKE_READY;
                    continue;
                }
            }

            if (state == State.TRANSITION_TO_SCORING_BASKET) {
                if (!reverting || (gamepad.right_bumper && !lastGamepad.right_bumper)) {
                    robot.raiseSlider();
                    if (robot.sliders.isInPosition() || gamepad.left_trigger > 0) {
                        robot.bucket.setPositionPreset("Ready");
                        state = State.SCORING_READY_BASKET;
                        timer1.reset();
                        continue;
                    }
                }

                if (reverting) {
                    robot.retractSlider();
                    robot.bucket.setPositionPreset("Transfer");

                    if (robot.sliders.isInPosition(20)) {
                        state = State.TRANSFERRING;
                        timer1.reset();
                    }
                }

                if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                    reverting = true;
                    timer2.reset();
                    continue;
                }
            }

            if (state == State.TRANSITION_TO_SCORING_CHAMBER) {
                if (!reverting || (gamepad.right_bumper && !lastGamepad.right_bumper)) {
                    robot.raiseSlider();
                    if (robot.sliders.isInPosition() || gamepad.left_trigger > 0) {
                        specimenScored = false;
                        state = State.SCORING_READY_CHAMBER;
                        timer1.reset();
                        continue;
                    }
                }

                if (reverting) {
                    robot.retractSlider();
                    if (robot.sliders.isInPosition(20)) {
                        state = State.INTAKE_READY;
                        timer1.reset();
                    }
                }

                if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                    reverting = true;
                    timer2.reset();
                    continue;
                }
            }

            if (state == State.SCORING_READY_BASKET) {
                robot.intake.pwmDisable();
                robot.raiseSlider();
                reverting = false;

                if (gamepad.right_trigger > 0) {
                    robot.bucket.setPositionPreset("Score");
                } else robot.bucket.setPositionPreset("Ready");

                if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                    state = State.TRANSITION_TO_SCORING_BASKET;
                    reverting = true;
                    timer1.reset();
                }

                if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                    state = State.RETURNING;
                    timer1.reset();
                }
            }

            if (state == State.SCORING_READY_CHAMBER) {
                robot.intake.pwmDisable();
                reverting = false;

                if (gamepad.right_trigger > 0) specimenScored = true;
                if (gamepad.left_trigger > 0) specimenScored = false;
                if (specimenScored) robot.confirmSpecimen(); else robot.raiseSlider();

                if (gamepad.left_bumper && !gamepad.left_bumper) {
                    state = State.TRANSITION_TO_SCORING_CHAMBER;
                    reverting = true;
                    timer1.reset();
                }

                if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                    state = State.RETURNING;
                    timer1.reset();
                    continue;
                }
            }

            if (state == State.RETURNING) {
                robot.intake.pwmEnable();
                robot.intake.setRaised();
                if (robot.scoringMode.equals("Basket")) intakeControls.call();

                if (reverting) {
                    state = State.TRANSITION_TO_SCORING_BASKET;
                    reverting = false;
                    continue;
                } else {
                    robot.retractSlider();
                    robot.bucket.setPositionPreset("Lift");
                    if (robot.sliders.isInPosition()) {
                        if (quickTransferQueued) state = State.TRANSFERRING;
                        else if (fullTransferQueued) state = State.TRANSFER_TRANSITION;
                        else state = State.INITIALISED;
                        timer1.reset();
                    }
                }

                if (gamepad.cross) fullTransferQueued = true;
                if (gamepad.triangle) quickTransferQueued = true;
                if (gamepad.square) {fullTransferQueued = false; quickTransferQueued = false;}

                if (gamepad.left_bumper && !lastGamepad.left_bumper) reverting = true;
            }

            if (gamepad.touchpad) robot.resetIMUYaw();
            if (operator.dpad_up) robot.scoringHeight = "High";
            if (operator.dpad_down) robot.scoringHeight = "Low";
//            if (operator.dpad_left) robot.scoringMode = "Bucket";
//            if (operator.dpad_right) robot.scoringMode = "Chamber";

            // Align to targets
            if (gamepad.dpad_up) target = 0.0;
            else if (gamepad.dpad_left) target = 135.0;
            else if (gamepad.dpad_right) target = 90.0;
            else if (gamepad.dpad_down) target = 180.0;
            else target = null;

            if (target != null) {
                double current = Math.toDegrees(heading);
                double smallerAngle = Math.min(
                        Math.abs(current - target),
                        360 - Math.abs(current - target)
                );

                double resultant1 = current - smallerAngle;
                if (resultant1 <= -180) resultant1 += 360;
                double resultant2 = current + smallerAngle;
                if (resultant2 > 180) resultant2 -= 360;

                if (resultant1 == target) pivot = Math.toRadians(smallerAngle);
                else if (resultant2 == target) pivot = Math.toRadians(-smallerAngle);
            }

            if (gamepad.share) robot.scoringMode = "Basket";
            if (gamepad.options) robot.scoringMode = "Chamber";

            robot.drivetrain.remote(vertical, horizontal, pivot, heading);
            telemetry.addData("STATE", state);
            telemetry.addData("MODE", robot.scoringMode + " | " + robot.scoringHeight);
            telemetry.addLine();
            telemetry.addData("Sliders", robot.sliders.getCurrentPosition());
            telemetry.update();
        }
    }

    public interface IntakeControls {void call();}

    enum State {
        INITIALISED,
        INTAKE_READY,
        TRANSFER_TRANSITION,
        TRANSFERRING,
        TRANSITION_TO_SCORING_BASKET,
        TRANSITION_TO_SCORING_CHAMBER,
        SCORING_READY_BASKET,
        SCORING_READY_CHAMBER,
        RETURNING
    }
}

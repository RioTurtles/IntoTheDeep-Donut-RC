package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class TeleoperatedV1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Project2Hardware robot = new Project2Hardware(hardwareMap);
        State state = State.INIT;
        Gamepad gamepad = new Gamepad();
        Gamepad operator = new Gamepad();
        Gamepad lastGamepad = new Gamepad();
        Gamepad lastOperator = new Gamepad();
        ElapsedTime timer1 = new ElapsedTime();
        ElapsedTime timer2 = new ElapsedTime();

        boolean reverting = false;
        boolean specimenScored = false;

        waitForStart();
        robot.resetIMUYaw();
        while (opModeIsActive()) {
            lastGamepad.copy(gamepad);
            lastOperator.copy(operator);
            gamepad.copy(gamepad1);
            operator.copy(gamepad2);
            double vertical = -gamepad.left_stick_y;
            double horizontal = gamepad.left_stick_x;
            double pivot = gamepad.right_stick_x;
            double heading = robot.getIMUYaw();

            if (state == State.INIT) {
                robot.armDown();
                robot.transferPosition();
                robot.linearExtension.setPositionPreset(false);

                if (gamepad.right_bumper) {
                    if (robot.scoringMode.equals("Basket")) {
                        robot.clawOpen();
                        robot.intakePosition();
                        robot.intakeOn();
                        state = State.INTAKE_READY;
                    } else {
                        timer1.reset();
                        state = State.INTAKE_TRANSITION_BETWEEN_MODES;
                    }
                    continue;
                }
            }

            if (state == State.INTAKE_READY) {
                specimenScored = false;

                if (robot.scoringMode.equals("Basket")) {
                    robot.sliders.setPower(0);
                    robot.armDown();
                    if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                        if (robot.intakeReversed) robot.intakeOn();
                        else {
                            if (robot.intakeOn) robot.intakeOff();
                            else robot.intakeOn();
                        }
                    }

                    if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                        if (robot.intakeUp) robot.intakePosition();
                        else robot.transferPosition();
                    }

                    if (gamepad.circle && !lastGamepad.circle) {
                        if (!robot.intakeReversed) robot.intakeReverse();
                        else {
                            if (robot.intakeOn) robot.intakeOff();
                            else robot.intakeReverse();
                        }
                    }

                    if (gamepad.options && !lastGamepad.options) {
                        robot.intakeSlow();
                        state = State.TRANSFER;
                        timer1.reset();
                    }

                    robot.linearExtension.setPositionPreset(gamepad.right_trigger > 0);
                } else if (robot.scoringMode.equals("Chamber")) {
                    robot.armUp();
                    if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                        if (robot.clawOpen) robot.clawClose(); else robot.clawOpen();
                    }

                    if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                        robot.clawClose();
                        robot.sliders.setPositionEncoder(robot.sliders.getCurrentPosition() + 400);
                        state = State.TRANSITION_TO_SCORING_PRE;
                        continue;
                    }
                }

                if (gamepad.triangle && !lastGamepad.triangle) {
                    state = State.INTAKE_TRANSITION_BETWEEN_MODES;
                    if (robot.scoringMode.equals("Basket")) robot.scoringMode = "Chamber";
                    else robot.scoringMode = "Basket";
                    timer1.reset();
                }
            }

            if (state == State.INTAKE_TRANSITION_BETWEEN_MODES) {
                robot.intakeOff();
                robot.transferPosition();
                robot.retractSlider();
                if (robot.scoringMode.equals("Basket")) {
                    if (timer1.milliseconds() > 500 && robot.sliders.isInPosition()) {
                        state = State.INTAKE_READY;
                        robot.clawOpen();
                        timer1.reset();
                    } else if (timer1.milliseconds() > 150) robot.armDown();
                    else robot.clawClose();
                } else {
                    if (timer1.milliseconds() > 500 && robot.sliders.isInPosition()) {
                        state = State.INTAKE_READY;
                        robot.clawOpen();
                        timer1.reset();
                    } else if (timer1.milliseconds() > 150) robot.armUp();
                    else robot.clawClose();
                }
            }

            if (state == State.TRANSFER) {
                robot.transferPosition();

                if (timer1.milliseconds() > 1650) {
                    if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                        reverting = false;
                        state = State.TRANSITION_TO_SCORING;
                    }

                    robot.intakeOff();
                }
                else if (timer1.milliseconds() > 1500) robot.clawClose();
                else if (timer1.milliseconds() > 800) robot.intakeReverse();
                else robot.clawOpen();

                if (gamepad.left_bumper) {
                    robot.clawOpen();
                    robot.intakePosition();
                    robot.intakeOn();
                    state = State.INTAKE_READY;
                    continue;
                }
            }

            if (state == State.TRANSITION_TO_SCORING_PRE) {
                if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                    reverting = false;
                    state = State.TRANSITION_TO_SCORING;
                    timer1.reset();
                }
            }

            if (state == State.TRANSITION_TO_SCORING) {
                if (!reverting || (gamepad.right_bumper && !lastGamepad.right_bumper)) {
                    robot.raiseSlider();
                    if (robot.sliders.isInPosition()) {
                        robot.armUp();
                        if (robot.scoringMode.equals("Basket")) state = State.SCORING_SAMPLE;
                        else state = State.SCORING_SPECIMEN;
                        timer1.reset();
                        continue;
                    }
                }

                if (reverting) {
                    robot.retractSlider();
                    robot.armDown();

                    if (robot.sliders.isInPosition()) {
                        if (robot.scoringMode.equals("Basket")) {
                            state = State.TRANSFER;
                            timer1.reset();
                        } else {
                            state = State.INTAKE_READY;
                            timer1.reset();
                        }
                    }
                }

                if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                    reverting = true;
                    timer2.reset();
                    continue;
                }
            }

            if (state == State.SCORING_SAMPLE) {
                robot.raiseSlider();
                reverting = false;

                if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                    if (robot.clawOpen) robot.clawClose();
                    else robot.clawOpen();
                }

                if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                    state = State.TRANSITION_TO_SCORING;
                    reverting = true;
                    timer1.reset();
                }

                if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                    state = State.RETURNING;
                    timer1.reset();
                }
            }

            if (state == State.SCORING_SPECIMEN) {
                reverting = false;
                if (!specimenScored) robot.raiseSlider();

                if (gamepad.left_bumper && !lastGamepad.left_bumper && !specimenScored) {
                    state = State.TRANSITION_TO_SCORING;
                    reverting = true;
                    timer1.reset();
                }

                if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                    specimenScored = true;
                    robot.confirmSpecimen();
                    timer1.reset();
                }

                if (specimenScored) {
                    if (robot.sliders.isInPosition()) {
                        robot.clawOpen();
                        if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                            state = State.RETURNING;
                            timer1.reset();
                        }
                    }
                }
            }

            if (state == State.RETURNING) {
                if (reverting) {
                    state = State.TRANSITION_TO_SCORING;
                    reverting = false;
                    continue;
                } else {
                    robot.retractSlider();
                    if (timer1.milliseconds() > 500 && robot.scoringMode.equals("Basket"))
                        robot.clawOpen();
                    else if (timer1.milliseconds() > 150) robot.armDown();
                    else robot.clawClose();
                    if (robot.sliders.isInPosition()) state = State.INIT;
                }

                if (gamepad.left_bumper && !lastGamepad.left_bumper
                        && robot.scoringMode.equals("Basket")) {
                    robot.clawClose();
                    reverting = true;
                }
            }

            if (gamepad.share) robot.sliders.forcePower(-1);
            else robot.sliders.setPositionEncoder(robot.sliders.getTargetPosition());

            if (gamepad.touchpad) robot.resetIMUYaw();
            if (operator.triangle) robot.scoringHeight = "High";
            if (operator.cross) robot.scoringHeight = "Low";
            if (operator.square) robot.scoringMode = "Basket";
            if (operator.circle) robot.scoringMode = "Chamber";

            robot.drivetrain.remote(vertical, horizontal, pivot, heading);

            telemetry.addData("State", state);
            telemetry.addLine();
            telemetry.addData("Mode", robot.scoringMode);
            telemetry.addData("Height", robot.scoringHeight);
            telemetry.addData("Best Colour", robot.bestColour());
            telemetry.addLine();
            telemetry.addData("Slider Height L", hardwareMap.get(DcMotorEx.class, "sliderL").getCurrentPosition());
            telemetry.addData("Slider Height R", hardwareMap.get(DcMotorEx.class, "sliderR").getCurrentPosition());
            telemetry.addData("Slider Target", robot.sliders.getTargetPosition());
            telemetry.addData("Slider Current", robot.sliders.getCurrentPosition());
            telemetry.update();
        }
    }

    public RevBlinkinLedDriver.BlinkinPattern getBlinkinPattern(String colour, State state) {
        // TODO: continue writing after tuning low chamber and testing rigging
        if (state == State.INIT) {

        }
        return null;
    }

    enum State {
        INIT,
        INTAKE_READY,
        INTAKE_TRANSITION_BETWEEN_MODES,
        TRANSFER,
        TRANSITION_TO_SCORING,
        TRANSITION_TO_SCORING_PRE,
        SCORING_SAMPLE,
        SCORING_SPECIMEN,
        RETURNING
    }
}

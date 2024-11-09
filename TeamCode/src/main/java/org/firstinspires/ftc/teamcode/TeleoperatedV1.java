package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp v1.2")
public class TeleoperatedV1 extends LinearOpMode {
    boolean specimenScored;

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

        specimenScored = false;
        boolean reverting = false;

        waitForStart();
        robot.resetIMUYaw();
        timer1.reset();

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
                        robot.intakeCoarse.setPwmEnable();
                        robot.intakeFine.setPwmEnable();
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
                        if (robot.intakeUp) {robot.intakePosition(); robot.intakeOn();}
                        else {robot.transferPosition(); robot.intakeOff();}
                    }

                    if (gamepad.circle && !lastGamepad.circle) {
                        if (robot.intakeReversed) robot.intakeOn();
                        else robot.intakeReverse();
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
                    robot.intakeCoarse.setPwmEnable();
                    robot.intakeFine.setPwmEnable();
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

                    if (robot.sliders.isInPosition(20)) {
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
                robot.intakeCoarse.setPosition(0.5);
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
                    if (robot.sliders.isInPosition()) {
                        state = State.INIT;
                        timer1.reset();
                    }
                }

                if (gamepad.left_bumper && !lastGamepad.left_bumper
                        && robot.scoringMode.equals("Basket")) {
                    robot.clawClose();
                    reverting = true;
                }
            }

            if (state == State.ASCENT) {
                robot.intakeOff();
                robot.transferPosition();
                robot.arm.setPositionPreset("Chamber");
                robot.sliders.setPositionPreset("Ascent");

                if (robot.sliders.isInPosition()) {
                    robot.arm.setPositionPreset("Chamber");
                    robot.clawOpen();
                }

//                if (gamepad.right_trigger > 0) robot.tilting.setPower(0.2);
//                else if (gamepad.left_trigger > 0) robot.tilting.setPower(-0.2);
//                else robot.tilting.setPower(0);
//
//                if (gamepad.right_bumper) {
//                    robot.sliders.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    robot.sliders.setPower(1);
//                } else if (gamepad.left_bumper) {
//                    robot.sliders.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    robot.sliders.setPower(-1);
//                } else {
//                    robot.sliders.setPositionEncoder(robot.sliders.getCurrentPosition());
//                }

                if (gamepad.triangle && !lastGamepad.triangle) {
                    robot.clawClose();
                    robot.retractSlider();
                    state = State.INIT;
                    timer1.reset();
                }
            }

            final double P = 1.5;
            if (operator.dpad_up) {
                pivot = Math.toRadians(Math.toDegrees(robot.getIMUYaw())) * P;
            } else if (operator.dpad_right) {
                pivot = Math.toRadians(Math.toDegrees(robot.getIMUYaw()) - 90) * P;
            } else if (operator.dpad_down) {
                pivot = Math.toRadians(Math.toDegrees(robot.getIMUYaw()) - 135) * P;
            }

            // Intentionally require both current and last gamepad.
            if (gamepad.share && lastGamepad.share) robot.sliders.forcePower(-1);
            else if (!gamepad.share && lastGamepad.share) robot.sliders.setPower(0);

            if (gamepad.touchpad) robot.resetIMUYaw();
            if (gamepad.square && !lastGamepad.square) {
                state = State.ASCENT;
                robot.clawClose();
                timer1.reset();
            }

            if (operator.triangle) robot.scoringHeight = "High";
            if (operator.cross) robot.scoringHeight = "Low";
            if (operator.square) robot.scoringMode = "Basket";
            if (operator.circle) robot.scoringMode = "Chamber";

            robot.drivetrain.remote(vertical, horizontal, pivot, heading);
            robot.leds.setPattern(getBlinkinPattern(robot.scoringMode, robot.bestColour(), state));

            telemetry.addData("State", state);
            telemetry.addLine();
            telemetry.addData("Mode", robot.scoringMode);
            telemetry.addData("Height", robot.scoringHeight);
            telemetry.addData("Light", robot.colour.getLightDetected());
            telemetry.addData("Best Colour", robot.bestColour());
            telemetry.addLine();
            telemetry.addData("Slider Height L", hardwareMap.get(DcMotorEx.class, "sliderL").getCurrentPosition());
            telemetry.addData("Slider Height R", hardwareMap.get(DcMotorEx.class, "sliderR").getCurrentPosition());
            telemetry.addData("Slider Target", robot.sliders.getTargetPosition());
            telemetry.addData("Slider Current", robot.sliders.getCurrentPosition());
            telemetry.update();
        }
    }

    public BlinkinPattern getBlinkinPattern(String mode, String colour, State state) {
        switch (state) {
            case INIT: return BlinkinPattern.GREEN;
            case INTAKE_READY:
                if (mode.equals("Basket")) {
                    switch (colour) {
                        case "Blue": return BlinkinPattern.HEARTBEAT_BLUE;
                        case "Red": return BlinkinPattern.HEARTBEAT_RED;
                        case "Yellow": return BlinkinPattern.HEARTBEAT_WHITE;
                        case "None": return BlinkinPattern.HEARTBEAT_GRAY;
                    }
                } else {
                    return BlinkinPattern.STROBE_WHITE;
                }
            case INTAKE_TRANSITION_BETWEEN_MODES: return BlinkinPattern.CONFETTI;

            case TRANSFER:
                switch (colour) {
                    case "Blue": return BlinkinPattern.BLUE;
                    case "Red": return BlinkinPattern.RED;
                    case "Yellow": return BlinkinPattern.YELLOW;
                }

            case TRANSITION_TO_SCORING_PRE:
            case TRANSITION_TO_SCORING:
                return BlinkinPattern.DARK_RED;

            case SCORING_SAMPLE:
                switch (colour) {
                    case "Blue": return BlinkinPattern.LIGHT_CHASE_BLUE;
                    case "Red": return BlinkinPattern.LIGHT_CHASE_RED;
                    case "Yellow": return BlinkinPattern.WHITE;
                }
            case SCORING_SPECIMEN:
                if (specimenScored) return BlinkinPattern.LIME;
                else return BlinkinPattern.COLOR_WAVES_FOREST_PALETTE;

            case RETURNING:
                return BlinkinPattern.VIOLET;

            default:
                return BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        }
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
        RETURNING,
        ASCENT
    }
}

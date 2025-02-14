package org.firstinspires.ftc.teamcode;


import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Project4Hardware.BucketPreset;
import org.firstinspires.ftc.teamcode.Project4Hardware.IntakeArmPreset;
import org.firstinspires.ftc.teamcode.Project4Hardware.LinearExtensionPreset;
import org.firstinspires.ftc.teamcode.Project4Hardware.ScoringMode;
import org.firstinspires.ftc.teamcode.Project4Hardware.SliderPreset;

import java.util.Objects;

@TeleOp(name="TeleOp v3.0")
public class TeleoperatedV3 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Project4Hardware robot = new Project4Hardware(hardwareMap);
        Gamepad gamepad = new Gamepad(), lastGamepad = new Gamepad();
        Gamepad operator = new Gamepad(), lastOperator = new Gamepad();
        MethodReference basicIntakeControls;
        State state = State.INIT;
        ElapsedTime timer1 = new ElapsedTime();

        waitForStart();
        while (opModeIsActive()) {
            lastGamepad.copy(gamepad); gamepad.copy(gamepad1);
            lastOperator.copy(operator); operator.copy(gamepad2);
            boolean left_bumper = gamepad.left_bumper && !lastGamepad.left_bumper;
            boolean right_bumper = gamepad.right_bumper && !lastGamepad.right_bumper;
            boolean left_trigger = gamepad.left_trigger > 0 && !(lastGamepad.left_trigger > 0);
            boolean right_trigger = gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0);
            double vertical = -gamepad.left_stick_y;
            double horizontal = gamepad.left_stick_x;
            double pivot = gamepad.right_stick_x;
            double heading = robot.getIMUYaw();

            basicIntakeControls = () -> {
                if (left_bumper) {if (robot.intakeOn) robot.intakeOff(); else robot.intakeOn();}

                if (right_bumper) {
                    if (robot.intakeUp) {
                        robot.setIntakeArm(IntakeArmPreset.INTAKE);
                        if (robot.linearExtension.currentPreset == LinearExtensionPreset.EXTENDED)
                            robot.setLinearExtension(LinearExtensionPreset.EXTENDED);
                        else robot.setLinearExtension(LinearExtensionPreset.INTAKE_RETRACTED);
                    } else {
                        robot.setIntakeArm(IntakeArmPreset.TRANSFER);
                        robot.setLinearExtension(LinearExtensionPreset.FULLY_RETRACTED);
                    }
                }

                if (gamepad.circle && !lastGamepad.circle) {
                    if (robot.intakeReversed) robot.intakeOn();
                    else robot.intakeReverse();
                }
            };

            if (state == State.INIT) {
                if (right_bumper) {
                    timer1.reset();
                    state = State.PRE_INTAKE_SAMPLE;
                }

                robot.retractSlider();
                robot.setBucket(BucketPreset.TRANSFER);
                robot.setIntakeArm(IntakeArmPreset.TRANSFER);
                robot.setLinearExtension(LinearExtensionPreset.FULLY_RETRACTED);
            }

            // SAMPLE INTAKE SUPERSTATE

            else if (state == State.PRE_INTAKE_SAMPLE) {
                robot.retractSlider();
                robot.setLinearExtension(LinearExtensionPreset.INTAKE_RETRACTED);
                robot.setIntakeArm(IntakeArmPreset.INTAKE);
                robot.intakeOn();
                state = State.INTAKE_SAMPLE;
            }

            else if (state == State.INTAKE_SAMPLE) {
                robot.setLinearExtension(LinearExtensionPreset.INTAKE_RETRACTED);
                robot.retractSlider();
                basicIntakeControls.call();

                // On start of right trigger hold
                if (gamepad.right_trigger > 0) state = State.INTAKE_EXTEND;

                if (gamepad.triangle) {
                    state = State.TRANSFER;
                    timer1.reset();
                }
            }

            else if (state == State.INTAKE_EXTEND) {
                basicIntakeControls.call();
                robot.setLinearExtension(LinearExtensionPreset.EXTENDED);

                // On release of right trigger
                if (!(gamepad.right_trigger > 0) && lastGamepad.right_trigger > 0) {
                    state = State.INTAKE_SAMPLE;
                }

                if (gamepad.right_bumper) {
                    state = State.TRANSFER_EXTEND;
                    timer1.reset();
                }
            }

            else if (state == State.TRANSFER_EXTEND) {
                if (timer1.milliseconds() > 1400)
                    state = State.TRANSFERRED_AWAIT_SAMPLE;
                if (timer1.milliseconds() > 400)  // Adjust transfer duration
                    robot.intakeOn();
                else {
                    robot.intakeOff();
                    robot.setLinearExtension(LinearExtensionPreset.FULLY_RETRACTED);
                    robot.setIntakeArm(IntakeArmPreset.TRANSFER);
                }

                if (left_bumper) {
                    state = State.TRANSFER_EXTEND;
                    robot.setIntakeArm(IntakeArmPreset.INTAKE);
                    robot.intakeOn();
                }
            }

            else if (state == State.TRANSFER) {
                if (timer1.milliseconds() > 1400)
                    state = State.TRANSFERRED_AWAIT_SAMPLE;
                if (timer1.milliseconds() > 400)  // Adjust transfer duration
                    robot.intakeOn();
                else {
                    robot.intakeOff();
                    robot.setLinearExtension(LinearExtensionPreset.FULLY_RETRACTED);
                    robot.setIntakeArm(IntakeArmPreset.TRANSFER);
                }

                if (left_bumper) {
                    state = State.INTAKE_SAMPLE;
                    robot.setIntakeArm(IntakeArmPreset.INTAKE);
                    robot.intakeOn();
                }
            }

            // SPECIMEN INTAKE SUPERSTATE
            else if (state == State.PRE_INTAKE_SPECIMEN) {
                robot.setSlider(SliderPreset.RETRACT_CHAMBER);
                robot.intakeOff();
                robot.setIntakeArm(IntakeArmPreset.TRANSFER);
                robot.setLinearExtension(LinearExtensionPreset.FULLY_RETRACTED);

                if (right_bumper) state = State.INTAKE_SPECIMEN;
            }

            else if (state == State.INTAKE_SPECIMEN) {
                robot.setSlider(SliderPreset.RETRACT_CHAMBER_LIFT);
                if (right_bumper) state = State.LIFT;
            }

            // TRANSFER SUPERSTATE
            else if (state == State.TRANSFERRED_AWAIT_SAMPLE) {
                robot.retractSlider();
                robot.setLinearExtension(LinearExtensionPreset.FULLY_RETRACTED);
                robot.setBucket(BucketPreset.TRANSFER);

                if (left_bumper) state = State.PRE_INTAKE_SAMPLE;
                if (right_bumper) state = State.LIFT;
                if (gamepad.circle) robot.intakeOn(); else robot.intakeOff();
            }

            else if (state == State.LIFT) {
                robot.raiseSlider();
            }

            if (gamepad.touchpad) robot.resetIMUYaw();
            if (gamepad.square || operator.square) {
                robot.scoringMode = ScoringMode.BASKET;
                state = state.counterpart();
            }
            if (gamepad.cross || operator.cross) {
                robot.scoringMode = ScoringMode.CHAMBER;
                state = state.counterpart();
            }

            robot.drivetrain.remote(vertical, horizontal, pivot, heading);
            telemetry.addData("State", state);
            telemetry.addLine();
            telemetry.addData("Intake Up", robot.intakeUp);
            telemetry.update();
        }
    }

    // Violating style guide for better readability.
    enum State {
        INIT,
        PRE_INTAKE_SAMPLE,
        PRE_INTAKE_SPECIMEN (PRE_INTAKE_SAMPLE),
        INTAKE_SAMPLE,
        INTAKE_EXTEND,
        INTAKE_SPECIMEN (INTAKE_SAMPLE),
        TRANSFER,
        TRANSFER_EXTEND,
        TRANSFERRED_AWAIT_SAMPLE,
        TRANSFERRED_AWAIT_SPECIMEN (TRANSFERRED_AWAIT_SAMPLE),
        LIFT;

        final @Nullable State counterpart;
        State(@Nullable State counterpart) {this.counterpart = counterpart;}
        State() {this.counterpart = null;}

        public State counterpart() {
            if (counterpart == null) {
                for (State constant : State.values()) {
                    if (Objects.nonNull(constant.counterpart())) {
                        if (constant.counterpart == this) return constant;
                    }
                }
                return this;
            } else return counterpart;
        }
    }

    interface MethodReference {void call();}
}

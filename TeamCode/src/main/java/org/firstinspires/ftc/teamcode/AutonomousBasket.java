package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous
public class AutonomousBasket extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Project3Hardware robot = new Project3Hardware(hardwareMap);
        PartialRoadrunnerHelper roadrunner = new PartialRoadrunnerHelper(drive, robot.drivetrain::remote);
        State state = State.PRELOAD;
        ElapsedTime autonomous = new ElapsedTime();
        ElapsedTime timer1 = new ElapsedTime();
        boolean intakeFinished = false;

        roadrunner.setPIDCoefficients(Axis.X, 0.1, 0, 0.0001);
        roadrunner.setPIDCoefficients(Axis.Y, 0.2, 0, 0.0001);
        roadrunner.setPIDCoefficients(Axis.HEADING, 0.8, 0.0001, 1);

        robot.linearExtension.setPositionPreset(false);

        waitForStart();
        autonomous.reset();
        roadrunner.setPoseEstimate(-33.14, -62.99, 270.00);
        while (opModeIsActive()) {
            if (state == State.PRELOAD) {
                roadrunner.setTarget(-58.54, -58.91, 225.00);
                robot.sliders.setPositionPreset("HighBasket");
                robot.bucket.setPositionPreset("Raised");

                if (robot.sliders.isInPosition(15) || autonomous.seconds() > 4) {
                    state = State.PRELOAD_SCORE;
                    timer1.reset();
                    robot.bucket.setPositionPreset("Ready");
                }
            }

            if (state == State.PRELOAD_SCORE) {

                if (timer1.milliseconds() > 500) {
                    robot.bucket.setPositionPreset("Transfer");
                    state = State.PATH_TO_FIRST;
                    timer1.reset();
                    robot.intake.setIntake();
                } else robot.bucket.setPositionPreset("Score");
            }

            if (state == State.PATH_TO_FIRST) {
                roadrunner.setTarget(-50.61, -42.28, 270.00);
                robot.retractSlider();

                if (roadrunner.getErrorHeadingDegrees() <= roadrunner.getTargetHeadingDegrees()) {
                    if (!intakeFinished) {robot.intakeOn(); robot.intake.setIntake();}
                    else robot.intakeOff();
                    if (timer1.milliseconds() > 4000) {
                        state = State.FIRST;
                        timer1.reset();
                        robot.intake.setRaised();
                    } else if (timer1.milliseconds() > 3250) robot.intakeReverse();
                    else if (timer1.milliseconds() > 1750) {
                        robot.linearExtension.setPositionPreset(false);
                        robot.intake.setTransfer();
                    } else if (timer1.milliseconds() > 1500) {
                        robot.intakeOff();
                        intakeFinished = true;
                    } else robot.intake.setIntake();
                } else timer1.reset();
            }

            if (state == State.FIRST) {
                roadrunner.setTarget(-58.54, -58.11, 225.00);
                robot.raiseSlider();
                intakeFinished = false;

                if (timer1.milliseconds() > 2500) {
                    robot.bucket.setPositionPreset("Transfer");
                    state = State.PATH_TO_SECOND;
                    timer1.reset();
                } else if (robot.sliders.isInPosition() || timer1.milliseconds() > 1800)
                    robot.bucket.setPositionPreset("Score");
                else robot.bucket.setPositionPreset("Ready");
            }

            if (state == State.PATH_TO_SECOND) {
                roadrunner.setTarget(-61.46, -41.48, 270.00);
                robot.retractSlider();

                if (roadrunner.getErrorHeadingDegrees() <= roadrunner.getTargetHeadingDegrees()) {
                    if (!intakeFinished) {robot.intakeOn(); robot.intake.setIntake();}
                    else robot.intakeOff();
                    if (timer1.milliseconds() > 4000) {
                        state = State.SECOND;
                        timer1.reset();
                        robot.intake.setRaised();
                        continue;
                    } else if (timer1.milliseconds() > 3250) robot.intakeReverse();
                    else if (timer1.milliseconds() > 1750) {
                        robot.linearExtension.setPositionPreset(false);
                        robot.intake.setTransfer();
                    } else if (timer1.milliseconds() > 1500) {
                        robot.intakeOff();
                        intakeFinished = true;
                    }
//                    else if (timer1.milliseconds() > 500)
//                        robot.linearExtension.setPositionPreset(true);
                } else timer1.reset();
            }

            if (state == State.SECOND) {
                roadrunner.setTarget(-60.54, -60.11, 225.00);
                robot.raiseSlider();
                intakeFinished = false;

                if (timer1.milliseconds() > 2500) {
                    robot.bucket.setPositionPreset("Transfer");
                    state = State.PATH_TO_THIRD;
                    timer1.reset();
                } else if (robot.sliders.isInPosition() || timer1.milliseconds() > 1750)
                    robot.bucket.setPositionPreset("Score");
                else robot.bucket.setPositionPreset("Ready");
            }

            if (state == State.PATH_TO_THIRD) {
                roadrunner.setTarget(-55.11, -39.65, 315.00);
                robot.retractSlider();

                if (roadrunner.getErrorHeadingDegrees() <= roadrunner.getTargetHeadingDegrees()) {
                    if (!intakeFinished) {robot.intakeOn(); robot.intake.setIntake();}
                    else robot.intakeOff();
                    if (timer1.milliseconds() > 4000) {
                        state = State.THIRD;
                        timer1.reset();
                        robot.intake.setRaised();
                        continue;
                    } else if (timer1.milliseconds() > 3250) robot.intakeReverse();
                    else if (timer1.milliseconds() > 1750) {
                        robot.linearExtension.setPositionPreset(false);
                        robot.intake.setTransfer();
                    } else if (timer1.milliseconds() > 1500) {
                        robot.intakeOff();
                        intakeFinished = true;
                    }
//                    else if (timer1.milliseconds() > 500)
//                        robot.linearExtension.setPosition(0.3);
                } else timer1.reset();
            }

            if (state == State.THIRD) {
                roadrunner.setTarget(-59.54, -59.11, 225.00);
                robot.raiseSlider();
                intakeFinished = false;

                if (timer1.milliseconds() > 2500) {
                    robot.intake.setRaised();
                    robot.bucket.setPositionPreset("Transfer");
                    state = State.PARK;
                    roadrunner.disablePowerLimit();
                    timer1.reset();
                } else if (robot.sliders.isInPosition() || timer1.milliseconds() > 1500)
                    robot.bucket.setPositionPreset("Score");
                else robot.bucket.setPositionPreset("Ready");
            }

            if (state == State.PARK) {
                robot.retractSlider();
                roadrunner.setTarget(-57.54, -57.54, 90.00);
                robot.bucket.setPositionPreset("Transfer");
                robot.intake.setRaised();
            }

            roadrunner.update();
            telemetry.addLine(roadrunner.toTelemetry());
            telemetry.addData("State", state);
            telemetry.addData("Sliders", robot.sliders.getCurrentPosition());
            telemetry.update();
        }
    }

    enum State {
        PRELOAD,
        PRELOAD_SCORE,
        PATH_TO_FIRST,
        FIRST,
        PATH_TO_SECOND,
        SECOND,
        PATH_TO_THIRD,
        THIRD,
        PARK
    }
}

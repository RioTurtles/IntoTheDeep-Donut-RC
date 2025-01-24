package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous
public class AutonomousRedBasket extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Project3Hardware robot = new Project3Hardware(hardwareMap);
        PartialRoadrunnerHelper roadrunner = new PartialRoadrunnerHelper(drive, robot.drivetrain::remote);
        State state = State.PRELOAD;
        ElapsedTime autonomous = new ElapsedTime();
        ElapsedTime timer1 = new ElapsedTime();

        roadrunner.setPIDCoefficients(Axis.X, 0.3, 0, 0.3);
        roadrunner.setPIDCoefficients(Axis.Y, 0.05, 0, 0.01);
        roadrunner.setPIDCoefficients(Axis.HEADING, 0.8, 0.0001, 1);

        robot.linearExtension.setPositionPreset(false);

        waitForStart();
        autonomous.reset();
        roadrunner.setPoseEstimate(-33.14, -62.99, 270.00);
        while (opModeIsActive()) {
            if (state == State.PRELOAD) {
                roadrunner.setTarget(-61.54, -56.11, 225.00);
                robot.sliders.setPositionPreset("HighBasket");
                robot.bucket.setPositionPreset("Raised");

                if (robot.sliders.isInPosition() || autonomous.seconds() > 4) {
                    state = State.PRELOAD_SCORE;
                    timer1.reset();
                    robot.bucket.setPositionPreset("Ready");
                }
            }

            if (state == State.PRELOAD_SCORE) {
                if (timer1.milliseconds() > 400) {
                    robot.bucket.setPositionPreset("Ready");
                    state = State.PATH_TO_FIRST;
                    timer1.reset();
                } else robot.bucket.setPositionPreset("Score");
            }

            if (state == State.PATH_TO_FIRST) {
                roadrunner.setTarget(-49.46, -46.48, 270.00);
                robot.retractSlider();

                if (roadrunner.getErrorHeadingDegrees() <= roadrunner.getTargetHeadingDegrees()) {
                    state = State.INTAKE_FIRST;
                    robot.intake.setIntake();
                    robot.intakeOn();
                    robot.linearExtension.setPositionPreset(true);
                    timer1.reset();
                }
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
        INTAKE_FIRST,
        PATH_TO_FIRST_SCORE
    }
}

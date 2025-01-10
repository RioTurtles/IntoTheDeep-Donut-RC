package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="RED Chamber")
public class AutonomousRedChamber extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Project3Hardware robot = new Project3Hardware(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(9.48, -62.99, Math.toRadians(90.00)))
                .lineTo(new Vector2d(4.60, -33.89))
                .lineToLinearHeading(new Pose2d(47.97, -59.23, Math.toRadians(270.00)))
                .lineToConstantHeading(new Vector2d(47.97, -63.18))
                .build();
    }
}

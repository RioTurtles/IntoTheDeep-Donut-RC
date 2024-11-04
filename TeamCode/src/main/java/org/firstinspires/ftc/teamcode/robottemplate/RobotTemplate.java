package org.firstinspires.ftc.teamcode.robottemplate;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class RobotTemplate {
    static HardwareMap hardwareMap;
    public RobotDrivetrain drivetrain;
    public IMU internalIMU;

    public RobotTemplate(HardwareMap hardwareMap) {RobotTemplate.hardwareMap = hardwareMap;}
    public void setDrivetrain(RobotDrivetrain drivetrain) {this.drivetrain = drivetrain;}

    public void initialiseIMU(
            String deviceName,
            RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection,
            RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection
    ) {
        internalIMU = RobotTemplate.hardwareMap.get(IMU.class, deviceName);
        internalIMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                logoFacingDirection, usbFacingDirection
        )));
    }

    public double getIMUYaw() {
        return internalIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void resetIMUYaw() {internalIMU.resetYaw();}
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.robottemplate.DualServoSystem;
import org.firstinspires.ftc.teamcode.robottemplate.DualMotorSystem;
import org.firstinspires.ftc.teamcode.robottemplate.RobotTemplate;
import org.firstinspires.ftc.teamcode.robottemplate.RobotDrivetrain;

public class Project2Hardware extends RobotTemplate {
    ServoImplEx claw;
    CRServoImplEx intakeL, intakeR;
    ColorRangeSensor colour;
    RevBlinkinLedDriver leds;

    DualMotorSystem sliders;
    DualMotorSystem tilting;
    DualServoSystem intakeFine;
    DualServoSystem intakeCoarse;
    DualServoSystem linearExtension;
    DualServoSystem arm;

    String scoringMode, scoringHeight;
    boolean intakeOn, intakeReversed, intakeUp, clawOpen;

    public Project2Hardware(HardwareMap hardwareMap) {
        super(hardwareMap);

        setDrivetrain(new RobotDrivetrain(
                "frontLeft", DcMotorSimple.Direction.FORWARD,
                "frontRight", DcMotorSimple.Direction.REVERSE,
                "backLeft", DcMotorSimple.Direction.FORWARD,
                "backRight", DcMotorSimple.Direction.REVERSE
        ));

        sliders = new DualMotorSystem("sliderL", "f", "sliderR", "r");
        tilting = new DualMotorSystem("tiltingL", "f", "tiltingR", "r");
        intakeCoarse = new DualServoSystem("intakeUL", "f", "intakeUR", "f");
        intakeFine = new DualServoSystem("intakeLL", "r", "intakeLR", "f");
        linearExtension = new DualServoSystem("extensionL", "r", "extensionR", "f");
        arm = new DualServoSystem("armL", "f", "armR", "r");
        intakeL = hardwareMap.get(CRServoImplEx.class, "intakeL");
        intakeR = hardwareMap.get(CRServoImplEx.class, "intakeR");
        claw = hardwareMap.get(ServoImplEx.class, "claw");
        colour = hardwareMap.get(ColorRangeSensor.class, "colour");
        leds = hardwareMap.get(RevBlinkinLedDriver.class, "leds");

        initialiseIMU("imu", LogoFacingDirection.RIGHT, UsbFacingDirection.UP);

        // Initialisation
        intakeL.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeR.setDirection(DcMotorSimple.Direction.REVERSE);
        claw.setDirection(Servo.Direction.FORWARD);

        sliders.addPreset("RetractBasket", 0);
        sliders.addPreset("RetractChamber", 1100);
        sliders.addPreset("LowBasket", 3400);
        sliders.addPreset("HighBasket", 5700);
        sliders.addPreset("LowChamber", 1600);
        sliders.addPreset("LowChamberL", 900);
        sliders.addPreset("HighChamber", 3900);
        sliders.addPreset("HighChamberL", 2900);

        linearExtension.addPreset(true, 0.5);
        linearExtension.addPreset(false, 0);

        intakeFine.addPreset("Intake", 0);
        intakeCoarse.addPreset("Intake", 0.075);
        intakeFine.addPreset("Transfer", 0.3);
        intakeCoarse.addPreset("Transfer", 0.4);
        
        arm.addPreset("Transfer", 0.02);
        arm.addPreset("Basket", 0.73);
        arm.addPreset("Chamber", 1);

        // Reset
        scoringMode = "Basket";
        scoringHeight = "High";
        sliders.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tilting.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void intakeOn() {
        intakeL.setPower(1);
        intakeR.setPower(1);
        intakeOn = true;
        intakeReversed = false;
    }

    public void intakeSlow() {
        intakeL.setPower(0.1);
        intakeR.setPower(0.1);
        intakeOn = true;
        intakeReversed = false;
    }

    public void intakeReverse() {
        intakeL.setPower(-0.4);
        intakeR.setPower(-0.4);
        intakeOn = true;
        intakeReversed = true;
    }

    public void intakeOff() {intakeL.setPower(0); intakeR.setPower(0); intakeOn = false;}
    public void clawOpen() {claw.setPosition(0.25); clawOpen = true;}
    public void clawClose() {claw.setPosition(0); clawOpen = false;}

    public void raiseSlider() {sliders.setPositionPreset(scoringHeight + scoringMode);}
    public void retractSlider() {sliders.setPositionPreset("Retract" + scoringMode);}
    public void confirmSpecimen() {sliders.setPositionPreset(scoringHeight + scoringMode + "L");}
    public void armUp() {arm.setPositionPreset(scoringMode);}
    public void armDown() {arm.setPositionPreset("Transfer");}

    public void intakePosition() {
        intakeCoarse.setPositionPreset("Intake");
        intakeFine.setPositionPreset("Intake");
        intakeUp = false;
    }
    
    public void transferPosition() {
        intakeFine.setPositionPreset("Transfer");
        intakeCoarse.setPositionPreset("Transfer");
        intakeUp = true;
    }

    public boolean intakeDetected() {return colour.getLightDetected() > 0.75;}

    public String bestColour() {
        double max = Math.max(colour.red(), Math.max(colour.green(), colour.blue()));
        if (max == colour.red()) return "Red";
        else if (max == colour.blue()) return "Blue";
        else return "Green";
    }
}

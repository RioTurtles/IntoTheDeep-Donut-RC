package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.robottemplate.DualMotorSystem;
import org.firstinspires.ftc.teamcode.robottemplate.DualServoSystem;
import org.firstinspires.ftc.teamcode.robottemplate.GettableEnum;
import org.firstinspires.ftc.teamcode.robottemplate.RobotDrivetrain;
import org.firstinspires.ftc.teamcode.robottemplate.RobotTemplate;

public class Project4Hardware extends RobotTemplate {
    ServoImplEx intakeL, intakeR;
    CRServoImplEx intake;
    DualMotorSystem sliders;
    DualServoSystem intakeArm;
    DualServoSystem linearExtension;
    DualServoSystem bucket;

    ScoringMode scoringMode;
    ScoringHeight scoringHeight;
    boolean intakeOn, intakeReversed, intakeUp, specimenConfirmed;

    public Project4Hardware(HardwareMap hardwareMap) {
        super(hardwareMap);
        setDrivetrain(new RobotDrivetrain(
                "frontLeft", DcMotorSimple.Direction.FORWARD,
                "frontRight", DcMotorSimple.Direction.REVERSE,
                "backLeft", DcMotorSimple.Direction.FORWARD,
                "backRight", DcMotorSimple.Direction.REVERSE
        ));
        drivetrain.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        initialiseIMU("imu", LogoFacingDirection.RIGHT, UsbFacingDirection.UP);

        intake = hardwareMap.get(CRServoImplEx.class, "intake");
        intakeArm = new DualServoSystem("intakeUL", "f", "intakeUR", "f");
        linearExtension = new DualServoSystem("extensionL", "r", "extensionR", "f");
        bucket = new DualServoSystem("armL", "f", "armR", "r");
        sliders = new DualMotorSystem("sliderL", "f", "sliderR", "r");

        intake.setDirection(DcMotorSimple.Direction.FORWARD);  // TODO: verify direction

//        sliders.addPreset(SliderPresets.BASKET_HIGH, 0);
//        sliders.addPreset(SliderPresets.RETRACT_CHAMBER, 410);
//        sliders.addPreset(SliderPresets.BASKET_LOW, 1700);
//        sliders.addPreset(SliderPresets.BASKET_HIGH, 2750);
//        sliders.addPreset("LowChamber", 830);
//        sliders.addPreset("LowChamberL", 200);
//        sliders.addPreset("HighChamber", 1950);
//        sliders.addPreset("HighChamberL", 1200);
//        sliders.addPreset("Ascent", 1900);
//        linearExtension.addPreset(true, 0.5);
//        linearExtension.addPreset(false, 0);
//        bucket.addPreset("Transfer", 0.0);
//        bucket.addPreset("Lift", 0.1);
//        bucket.addPreset("Raised", 0.1);
//        bucket.addPreset("Ready", 0.15);
//        bucket.addPreset("Score", 0.55);

        sliders.addPresets(SliderPreset.class);
        intakeArm.addPresets(IntakeArmPreset.class);
        linearExtension.addPresets(LinearExtensionPreset.class);
        bucket.addPresets(BucketPreset.class);

        // Reset
        scoringMode = ScoringMode.BASKET;
        scoringHeight = ScoringHeight.HIGH;
        specimenConfirmed = false;
        sliders.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public enum ScoringMode {BASKET, CHAMBER}
    public enum ScoringHeight {HIGH, LOW, ASCENT, RETRACT}

    public enum SliderPreset implements GettableEnum {
        RETRACT_BASKET(0),
        RETRACT_CHAMBER(410),
        RETRACT_CHAMBER_LIFT(580),
        BASKET_LOW(1700),
        BASKET_HIGH(2750),
        CHAMBER_LOW(830),
        CHAMBER_LOW_SCORE(200),
        CHAMBER_HIGH(1950),
        CHAMBER_HIGH_SCORE(1200),
        ASCENT(1900),
        ASCENT_INVERSE(-((int) ASCENT.get()));

        private final int value;
        SliderPreset(int value) {this.value = value;}
        public Object get() {return value;}
    }

    public enum IntakeArmPreset implements GettableEnum {
        INTAKE(0.67),
        CLEARANCE(0.8),
        TRANSFER(1.0);

        private final double value;
        IntakeArmPreset(double value) {this.value = value;}
        @Override public Object get() {return value;}
    }

    public enum LinearExtensionPreset implements GettableEnum {
        EXTENDED(0.5),
        INTAKE_RETRACTED(0.15),
        FULLY_RETRACTED(0);

        private final double value;
        LinearExtensionPreset(double value) {this.value = value;}
        public Object get() {return value;}
    }

    public enum BucketPreset implements GettableEnum {
        TRANSFER(0.05),
        LIFT(0.2),
        RAISED(0.2),
        READY(0.25),
        SCORE(0.65);

        private final double value;
        BucketPreset(double value) {this.value = value;}
        public Object get() {return value;}
    }

    public void setSlider(SliderPreset v) {sliders.setPositionPreset(v);}
    public void setLinearExtension(LinearExtensionPreset v) {linearExtension.setPositionPreset(v);}
    public void setBucket(BucketPreset v) {bucket.setPositionPreset(v);}

    public void setIntakeArm(IntakeArmPreset v) {
        switch (v) {
            case INTAKE: intakeUp = false; break;
            case TRANSFER: intakeUp = true; break;
        }
        intakeArm.setPositionPreset(v);
    }

    public void intakeOn() {
        intake.setPower(1);
        intakeOn = true;
        intakeReversed = false;
    }

    public void intakeReverse() {
        intake.setPower(-0.4);
        intakeOn = true;
        intakeReversed = true;
    }

    public void intakeOff() {
        intake.setPower(0);
        intakeOn = false;
        intakeReversed = false;
    }

    public SliderPreset getSliderPreset() {
        String r;
        if (specimenConfirmed) r = "_SCORE"; else r = "";

        switch (scoringHeight) {
            case HIGH:
            case LOW:
                return SliderPreset.valueOf(scoringMode + "_" + scoringHeight + r);
            case RETRACT: return SliderPreset.valueOf("RETRACT_" + scoringMode);
            case ASCENT: return SliderPreset.ASCENT;
            default: return SliderPreset.RETRACT_BASKET;
        }
    }

    public void raiseSlider() {
        sliders.setPositionPreset(getSliderPreset());
        sliders.setSpeed(1);
        if (scoringHeight == ScoringHeight.RETRACT) sliders.setSpeed(0);
    }

    public void retractSlider() {
        sliders.setPositionPreset(SliderPreset.valueOf("RETRACT_" + scoringMode));
    }

    public void powerSlider() {sliders.setSpeed(1);}
    public void unpowerSlider() {sliders.setSpeed(0);}

    public void confirmSpecimen() {specimenConfirmed = true;}
    public void denySpecimen() {specimenConfirmed = false;}
}

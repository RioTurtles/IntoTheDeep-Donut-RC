package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.robottemplate.DualMotorSystem;
import org.firstinspires.ftc.teamcode.robottemplate.DualServoSystem;
import org.firstinspires.ftc.teamcode.robottemplate.RobotDrivetrain;
import org.firstinspires.ftc.teamcode.robottemplate.RobotTemplate;

public class Project3Hardware extends RobotTemplate {
    private final ServoImplEx intakeUL, intakeUR, intakeLL, intakeLR;
    CRServoImplEx intakeL, intakeR;
    ColorRangeSensor colour;

    IntakeModule intake;
    DualMotorSystem sliders;
    DualServoSystem linearExtension;
    DualServoSystem bucket;

    String scoringMode, scoringHeight;
    boolean intakeOn, intakeReversed, intakeUp;

    public Project3Hardware(HardwareMap hardwareMap) {
        super(hardwareMap);
        setDrivetrain(new RobotDrivetrain(
                "frontLeft", DcMotorSimple.Direction.FORWARD,
                "frontRight", DcMotorSimple.Direction.REVERSE,
                "backLeft", DcMotorSimple.Direction.FORWARD,
                "backRight", DcMotorSimple.Direction.REVERSE
        ));
        drivetrain.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        initialiseIMU("imu", LogoFacingDirection.RIGHT, UsbFacingDirection.UP);

        intakeUL = hardwareMap.get(ServoImplEx.class, "intakeUL");
        intakeUR = hardwareMap.get(ServoImplEx.class, "intakeUR");
        intakeLL = hardwareMap.get(ServoImplEx.class, "intakeLL");
        intakeLR = hardwareMap.get(ServoImplEx.class, "intakeLR");
        intakeUL.setDirection(Servo.Direction.FORWARD);
        intakeUR.setDirection(Servo.Direction.FORWARD);
        intakeLL.setDirection(Servo.Direction.REVERSE);
        intakeLR.setDirection(Servo.Direction.FORWARD);
        intakeL = hardwareMap.get(CRServoImplEx.class, "intakeL");
        intakeR = hardwareMap.get(CRServoImplEx.class, "intakeR");
        colour = hardwareMap.get(ColorRangeSensor.class, "colour");
        intake = new IntakeModule("transfer");

        sliders = new DualMotorSystem("sliderL", "f", "sliderR", "r");
        linearExtension = new DualServoSystem("extensionL", "r", "extensionR", "f");
        bucket = new DualServoSystem("armL", "f", "armR", "r");

        // Initialisation
        intakeL.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeR.setDirection(DcMotorSimple.Direction.REVERSE);

        sliders.addPreset("RetractBasket", 0);
        sliders.addPreset("RetractChamber", 410);
        sliders.addPreset("LowBasket", 1700);
        sliders.addPreset("HighBasket", 2750);
        sliders.addPreset("LowChamber", 830);
        sliders.addPreset("LowChamberL", 200);
        sliders.addPreset("HighChamber", 1950);
        sliders.addPreset("HighChamberL", 1200);
        sliders.addPreset("Ascent", 1900);

        linearExtension.addPreset(true, 0.5);
        linearExtension.addPreset(false, 0);
        
        bucket.addPreset("Transfer", 0.0);
        bucket.addPreset("Lift", 0.1);
        bucket.addPreset("Raised", 0.1);
        bucket.addPreset("Ready", 0.15);
        bucket.addPreset("Score", 0.55);

        // Reset
        scoringMode = "Basket";
        scoringHeight = "High";
        sliders.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public class IntakeModule {
        private String current;
        public IntakeModule(String currentPosition) {this.current = currentPosition;}
        public boolean currentStateEq(String str) {return str.equalsIgnoreCase(current);}
        final double offsetL = 0.0;
        final double offsetR = 0.0;

        @Deprecated
        public void unpack(double _LL, double _LR, double _UL, double _UR) {
            intakeLL.setPosition(_LL);
            intakeLR.setPosition(_LR);
            intakeUL.setPosition(_UL);
            intakeUR.setPosition(_UR);
        }

        @Deprecated
        private void unpack(double _LL, double _LR, double _UL, double _UR, String s, boolean u) {
            intakeLL.setPosition(_LL);
            intakeLR.setPosition(_LR);
            intakeUL.setPosition(_UL);
            intakeUR.setPosition(_UR);
            current = s;
            intakeUp = u;
        }

        private void unpack(double _L, double _R, String s, boolean u) {
            intakeUL.setPosition(_L);
            intakeUR.setPosition(_R);
            current = s;
            intakeUp = u;
        }

        private void synchronous(double p, String s, boolean u) {
            unpack(p + offsetL, p + offsetR, s, u);
        }

        public void pwmEnable() {
            intakeUL.setPwmEnable();
            intakeUR.setPwmEnable();
            intakeLL.setPwmEnable();
            intakeLR.setPwmEnable();
        }

        public void pwmDisable() {
            intakeUL.setPwmDisable();
            intakeUR.setPwmDisable();
            intakeLL.setPwmDisable();
            intakeLR.setPwmDisable();
        }

        public void setIntake() {synchronous(0.05, "intake", false);}
        public void setExtended() {synchronous(0.06, "extended", false);}
        public void setTransfer() {synchronous(0.56, "transfer", true);}
        public void setRaised() {synchronous(0.5, "raised", true);}
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

    public void raiseSlider() {sliders.setPositionPreset(scoringHeight + scoringMode);}
    public void retractSlider() {sliders.setPositionPreset("Retract" + scoringMode);}
    public void confirmSpecimen() {sliders.setPositionPreset(scoringHeight + scoringMode + "L");}

    public boolean intakeDetected() {return colour.getLightDetected() > 0.75;}

    public String bestColour() {
        if (intakeDetected()) {
            double max = Math.max(colour.red(), Math.max(colour.green(), colour.blue()));
            if (max == colour.red()) return "Red";
            else if (max == colour.blue()) return "Blue";
            else return "Yellow";
        } else return "None";
    }
}

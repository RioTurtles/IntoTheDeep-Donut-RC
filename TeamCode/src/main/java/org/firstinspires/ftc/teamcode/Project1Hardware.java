package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;


public class Project1Hardware {
    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    DcMotorEx sliderL, sliderR;
    CRServoImplEx intakeL, intakeR;
    ServoImplEx intakeUL, intakeUR, intakeLL, intakeLR;
    ServoImplEx extensionL, extensionR;
    ServoImplEx armL, armR;
    ServoImplEx claw;
    IMU imu;
    RevBlinkinLedDriver lights;

    Drivetrain drivetrain;
    Intake intake;

    boolean armUp;
    boolean intakeOn;
    boolean intakeExtended;
    boolean intakeUp;
    boolean clawOpen;

    public Project1Hardware(@NonNull HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        sliderL = hardwareMap.get(DcMotorEx.class, "sliderL");
        sliderR = hardwareMap.get(DcMotorEx.class, "sliderR");
        armL = hardwareMap.get(ServoImplEx.class, "armL");
        armR = hardwareMap.get(ServoImplEx.class, "armR");
        intakeL = hardwareMap.get(CRServoImplEx.class, "intakeL");
        intakeR = hardwareMap.get(CRServoImplEx.class, "intakeR");
        intakeUL = hardwareMap.get(ServoImplEx.class, "intakeUL");
        intakeUR = hardwareMap.get(ServoImplEx.class, "intakeUR");
        intakeLL = hardwareMap.get(ServoImplEx.class, "intakeLL");
        intakeLR = hardwareMap.get(ServoImplEx.class, "intakeLR");
        extensionL = hardwareMap.get(ServoImplEx.class, "extensionLeft");
        extensionR = hardwareMap.get(ServoImplEx.class, "extensionRight");
        claw = hardwareMap.get(ServoImplEx.class, "claw");
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "led");

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));

        drivetrain = new Drivetrain();
        intake = new Intake();
    }

    /** This class represents the robot's drivetrain. */
    class Drivetrain {
        protected DcMotor frontLeft, frontRight, backLeft, backRight;
        double max, sin, cos, theta, power, vertical, horizontal, pivot, heading;
        double FLPower, FRPower, BLPower, BRPower;

        public Drivetrain() {
            this.frontLeft = Project1Hardware.this.frontLeft;
            this.frontRight = Project1Hardware.this.frontRight;
            this.backLeft = Project1Hardware.this.backLeft;
            this.backRight = Project1Hardware.this.backRight;
        }

        /**
         * Classic drivetrain movement method - self explanatory.
         * @param vertical Gamepad's vertical axis (y).
         * @param horizontal Gamepad's horizontal axis (x).
         * @param pivot Gamepad's rotational axis (<code>right_stick_x</code>).
         * @param heading Robot's heading.
         */
        public void remote(double vertical, double horizontal, double pivot, double heading) {
            this.vertical = vertical;
            this.horizontal = horizontal;
            this.pivot = pivot;
            this.heading = heading ;

            theta = 2 * Math.PI + Math.atan2(vertical,horizontal) - heading;
            power = Math.hypot(horizontal, vertical);

            sin = Math.sin(theta - Math.PI/4);
            cos = Math.cos(theta - Math.PI/4);
            max = Math.max(Math.abs(sin), Math.abs(cos));

            FLPower = power * (cos/max) + pivot;
            FRPower = power * sin/max - pivot;
            BLPower = power * -(sin/max) - pivot;
            BRPower = power * -(cos/max) + pivot;

            this.frontLeft.setPower(-FLPower);
            this.frontRight.setPower(-FRPower);
            this.backLeft.setPower(BLPower);
            this.backRight.setPower(BRPower);

            this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        /** Also mecanum drive ({@link #remote(double, double, double, double) remote()}) but more
         * organised.
         * @param vertical Gamepad's vertical axis (y).
         * @param horizontal Gamepad's horizontal axis (x).
         * @param pivot Gamepad's rotational axis (<code>right_stick_x</code>).
         * @param heading Robot's heading.
         */
        public void remote2(double vertical, double horizontal, double pivot, double heading) {
            this.frontLeft.setDirection(DcMotor.Direction.FORWARD);
            this.frontRight.setDirection(DcMotor.Direction.REVERSE);
            this.backLeft.setDirection(DcMotor.Direction.FORWARD);
            this.backRight.setDirection(DcMotor.Direction.REVERSE);

            this.vertical = vertical;
            this.horizontal = horizontal;
            this.pivot = pivot;
            this.heading = heading + (Math.PI/2);

            theta = 2 * Math.PI + Math.atan2(vertical, horizontal) - heading;
            power = Math.hypot(horizontal, vertical);

            sin = Math.sin(theta - Math.PI/4);
            cos = Math.cos(theta - Math.PI/4);
            max = Math.max(Math.abs(sin), Math.abs(cos));

            /*
                FLPower = power * (cos/max) + pivot;
                FRPower = power * (sin/max) - pivot;
                BLPower = power * (sin/max) + pivot;
                BRPower = power * (cos/max) - pivot;
            */

            FLPower = power * (cos/max) - pivot;
            FRPower = power * (sin/max) + pivot;
            BLPower = power * (sin/max) - pivot;
            BRPower = power * (cos/max) + pivot;

            this.frontLeft.setPower(FLPower);
            this.frontRight.setPower(FRPower);
            this.backLeft.setPower(BLPower);
            this.backRight.setPower(BRPower);

            this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        /** Undocumented - copied from MecanumDrive.java */
        public void part1(double theta, double pivot, double power) {
            theta = 2 * Math.PI + (theta / 360 * 2 * Math.PI) - Math.PI / 2;

            sin = Math.sin(theta - Math.PI/4);
            cos = Math.cos(theta - Math.PI/4);
            max = Math.max(Math.abs(sin), Math.abs(cos));

            FLPower = power * (cos/max) + pivot;
            FRPower = power * sin/max - pivot;
            BLPower = power * -(sin/max) - pivot;
            BRPower = power * -(cos/max) + pivot;

            this.frontLeft.setPower(-FLPower);
            this.frontRight.setPower(-FRPower);
            this.backLeft.setPower(BLPower);
            this.backRight.setPower(BRPower);

            this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        /** Undocumented - copied from MecanumDrive.java */
        public void drive(double target, double power, double pivot, double distance) {

            this.theta = Math.PI + (target * Math.PI/180);
            sin = Math.sin(theta - Math.PI/4);
            cos = Math.cos(theta - Math.PI/4);
            max = Math.max(Math.abs(sin), Math.abs(cos));

            int FL = this.frontLeft.getCurrentPosition();
            int FR = this.frontRight.getCurrentPosition();
            int BL = this.backLeft.getCurrentPosition();
            int BR = this.backRight.getCurrentPosition();

            double orig = FL;
            double cur = orig;

            while (Math.abs(cur - orig) <= distance) {
                FL = this.frontLeft.getCurrentPosition();
                FR = this.frontRight.getCurrentPosition();
                BL = this.backLeft.getCurrentPosition();
                BR = this.backRight.getCurrentPosition();

                cur = FL;

                FLPower = power * -(cos/max) + pivot;
                FRPower = power * sin/max + pivot;
                BLPower = power * -(sin/max) + pivot;
                BRPower = power * cos/max + pivot;

                this.frontLeft.setPower(-FLPower);
                this.frontRight.setPower(-FRPower);
                this.backLeft.setPower(BLPower);
                this.backRight.setPower(BRPower);

                this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
    }

    class Intake {
        ServoImplEx upperLeft, upperRight;
        ServoImplEx lowerLeft, lowerRight;
        CRServoImplEx intakeLeft, intakeRight;
        ServoImplEx extensionL, extensionR;

        private double height;
        private double coarsePitch, finePitch;

        final double INTAKE_HEIGHT = 0;
        final double INTAKE_PITCH_COARSE = 0;
        final double INTAKE_PITCH_FINE = 0;

        final double TRANSFER_HEIGHT = 0.2;
        final double TRANSFER_PITCH_COARSE = 0.2;
        final double TRANSFER_PITCH_FINE = 0.3;

        final double EXTENSION_LEFT = 1;
        final double EXTENSION_RIGHT = 1;
        final double RETRACT_LEFT = 0;
        final double RETRACT_RIGHT = 0;

        public Intake() {
            this.upperLeft = Project1Hardware.this.intakeUL;
            this.upperRight = Project1Hardware.this.intakeUR;
            this.lowerLeft = Project1Hardware.this.intakeLL;
            this.lowerRight = Project1Hardware.this.intakeLR;
            this.intakeLeft = Project1Hardware.this.intakeL;
            this.intakeRight = Project1Hardware.this.intakeR;
            this.extensionL = Project1Hardware.this.extensionL;
            this.extensionR = Project1Hardware.this.extensionR;
        }

        private void setUpperPair(double position) {
            this.upperLeft.setPosition(position);
            this.lowerLeft.setPosition(position);
        }

        private void setLowerPair(double position) {
            this.lowerLeft.setPosition(position);
            this.lowerRight.setPosition(position);
        }

        private void apply() {
            setUpperPair(height + coarsePitch);
            setLowerPair(height + finePitch);
        }

        public void setIntakePosition() {
            setHeight(INTAKE_HEIGHT);
            setCoarsePitch(INTAKE_PITCH_COARSE);
            setFinePitch(INTAKE_PITCH_FINE);
            Project1Hardware.this.intakeUp = true;
        }

        public void setTransferPosition() {
            setHeight(TRANSFER_HEIGHT);
            setCoarsePitch(TRANSFER_PITCH_COARSE);
            setFinePitch(TRANSFER_PITCH_FINE);
            intakeUp = false;
        }

        public void setHeight(double height) {
            this.height = height;
            apply();
        }

        public void setCoarsePitch(double pitch) {
            this.coarsePitch = pitch;
            apply();
        }

        public void setFinePitch(double pitch) {
            this.finePitch = pitch;
            apply();
        }

        public double getHeight() {return height;}
        public double getCoarsePitch() {return coarsePitch;}
        public double getFinePitch() {return finePitch;}

        public double getTotalPitch() {
            return Range.clip(coarsePitch + finePitch, 0, 1);
        }

        public void on() {
            intakeLeft.setPower(1);
            intakeRight.setPower(1);
            Project1Hardware.this.intakeOn = true;
        }

        public void off() {
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
            Project1Hardware.this.intakeOn = false;
        }

        public void extend() {
            extensionL.setPosition(EXTENSION_LEFT);
            extensionR.setPosition(EXTENSION_RIGHT);
            Project1Hardware.this.intakeExtended = true;
        }

        public void retract() {
            extensionL.setPosition(RETRACT_LEFT);
            extensionR.setPosition(RETRACT_RIGHT);
            Project1Hardware.this.intakeExtended = false;
        }
    }

    class SliderSystem {
        DcMotorEx left, right;
        private DcMotor.RunMode mode;
        int targetPosition;

        public SliderSystem() {
            left = Project1Hardware.this.sliderL;
            right = Project1Hardware.this.sliderR;
        }

        public void stopAndResetEncoders(DcMotor.RunMode nextMode) {
            left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            left.setMode(nextMode);
            right.setMode(nextMode);
        }

        public void stopAndResetEncoders() {stopAndResetEncoders(mode);}

        public void setPosition(int position) {
            targetPosition = position;
            left.setTargetPosition(position);
            right.setTargetPosition(position);
            left.setPower(1);
            right.setPower(1);
            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public int getCurrentPosition() {return left.getCurrentPosition();}

        public void setHighBasket() {setPosition(600);}
        public void setLowBasket() {setPosition(300);}
        public void setHighRung() {setPosition(500);}
        public void setLowRung() {setPosition(200);}
        public void confirmSpecimen() {setPosition(getCurrentPosition() - 50);}

        public void force(double speed) {
            left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            left.setPower(speed);
            right.setPower(speed);
        }
    }

    public void setArmTransferPosition() {
        armL.setPosition(0);
        armR.setPosition(0);
        armUp = false;
    }

    public void setArmScoringPosition() {
        armL.setPosition(1);
        armR.setPosition(1);
        armUp = true;
    }

    public void openClaw() {claw.setPosition(0.2); clawOpen = true;}
    public void closeClaw() {claw.setPosition(0); clawOpen = false;}

    public double getIMUYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw();
    }
}
package org.firstinspires.ftc.teamcode.robottemplate;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/** This class represents the robot's drivetrain. */
public class RobotDrivetrain implements RobotSubsystemTemplate {
    protected DcMotor frontLeft, frontRight, backLeft, backRight;
    private double max, sin, cos, theta, power;
    private double powerFL, powerFR, powerBL, powerBR;

    /**
     * Constructor for the drivetrain class using {@link DcMotor} objects.
     * @param fL Object of the front left motor.
     * @param fR Object of the front right motor.
     * @param bL Object of the back left motor.
     * @param bR Object of the back right motor.
     */
    public RobotDrivetrain(DcMotor fL, DcMotor fR, DcMotor bL, DcMotor bR) {
        this.frontLeft = fL;
        this.frontRight = fR;
        this.backLeft = bL;
        this.backRight = bR;
    }

    /**
     * Constructor for the drivetrain class using actuator names.
     * @param nameFL Actuator name of the front left motor.
     * @param nameFR Actuator name of the front right motor.
     * @param nameBL Actuator name of the back left motor.
     * @param nameBR Actuator name of the back right motor.
     */
    public RobotDrivetrain(
            String nameFL, DcMotorSimple.Direction directionFL,
            String nameFR, DcMotorSimple.Direction directionFR,
            String nameBL, DcMotorSimple.Direction directionBL,
            String nameBR, DcMotorSimple.Direction directionBR
    ) {
        this.frontLeft = RobotTemplate.hardwareMap.get(DcMotor.class, nameFL);
        this.frontRight = RobotTemplate.hardwareMap.get(DcMotor.class, nameFR);
        this.backLeft = RobotTemplate.hardwareMap.get(DcMotor.class, nameBL);
        this.backRight = RobotTemplate.hardwareMap.get(DcMotor.class, nameBR);
        setMotorDirections(directionFL, directionFR, directionBL, directionBR);
    }

    /**
     * Sets {@link DcMotorSimple.Direction}s for the drivetrain.
     * @param fL Direction of the front left motor.
     * @param fR Direction of the front right motor.
     * @param bL Direction of the back left motor.
     * @param bR Direction of the back right motor.
     */
    public void setMotorDirections(DcMotorSimple.Direction fL, DcMotorSimple.Direction fR, DcMotorSimple.Direction bL, DcMotorSimple.Direction bR) {
        frontLeft.setDirection(fL);
        frontRight.setDirection(fR);
        backLeft.setDirection(bL);
        backRight.setDirection(bR);
    }

    /**
     * Sets the {@link DcMotor.RunMode} for all 4 motors.
     * @param mode The mode to set to.
     */
    public void setMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backRight.setMode(mode);
        backRight.setMode(mode);
    }

    /**
     * Sets the {@link DcMotor.ZeroPowerBehavior} for all 4 motors.
     * @param setting The {@link DcMotor.ZeroPowerBehavior}.
     */
    public void setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior setting) {
        frontLeft.setZeroPowerBehavior(setting);
        frontLeft.setZeroPowerBehavior(setting);
        backLeft.setZeroPowerBehavior(setting);
        backRight.setZeroPowerBehavior(setting);
    }

    /**
     * Classic drivetrain movement method - self explanatory.
     * @param vertical Gamepad's vertical axis (y).
     * @param horizontal Gamepad's horizontal axis (x).
     * @param pivot Gamepad's rotational axis (<code>right_stick_x</code>).
     * @param heading Robot's heading.
     */
    public void remote(double vertical, double horizontal, double pivot, double heading) {
        theta = 2 * Math.PI + Math.atan2(vertical, horizontal) - heading;
        power = Math.hypot(horizontal, vertical);

        sin = Math.sin(theta - Math.PI/4);
        cos = Math.cos(theta - Math.PI/4);
        max = Math.max(Math.abs(sin), Math.abs(cos));

        powerFL = power * (cos/max) + pivot;
        powerFR = power * sin/max - pivot;
        powerBL = power * -(sin/max) - pivot;
        powerBR = power * -(cos/max) + pivot;

        this.frontLeft.setPower(-powerFL);
        this.frontRight.setPower(-powerFR);
        this.backLeft.setPower(powerBL);
        this.backRight.setPower(powerBR);

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
        heading += (Math.PI/2);

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

        powerFL = power * (cos/max) - pivot;
        powerFR = power * (sin/max) + pivot;
        powerBL = power * (sin/max) - pivot;
        powerBR = power * (cos/max) + pivot;

        this.frontLeft.setPower(powerFL);
        this.frontRight.setPower(powerFR);
        this.backLeft.setPower(powerBL);
        this.backRight.setPower(powerBR);

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

        powerFL = power * (cos/max) + pivot;
        powerFR = power * sin/max - pivot;
        powerBL = power * -(sin/max) - pivot;
        powerBR = power * -(cos/max) + pivot;

        this.frontLeft.setPower(-powerFL);
        this.frontRight.setPower(-powerFR);
        this.backLeft.setPower(powerBL);
        this.backRight.setPower(powerBR);

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

            powerFL = power * -(cos/max) + pivot;
            powerFR = power * sin/max + pivot;
            powerBL = power * -(sin/max) + pivot;
            powerBR = power * cos/max + pivot;

            this.frontLeft.setPower(-powerFL);
            this.frontRight.setPower(-powerFR);
            this.backLeft.setPower(powerBL);
            this.backRight.setPower(powerBR);

            this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
}
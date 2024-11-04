package org.firstinspires.ftc.teamcode.robottemplate;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.HashMap;
import java.util.Locale;
import java.util.Map;

public class SingleMotorSystem implements RobotSubsystemTemplate {
    final private DcMotor motor;
    private double speed;
    private int tolerance;
    Map<Object, Integer> presets;
    public Object currentPreset;

    public SingleMotorSystem(DcMotor motor) {
        presets = new HashMap<>();
        this.motor = motor;
        setSpeed(1);
        setTolerance(5);
    }

    public SingleMotorSystem(String motorName, String direction) {
        presets = new HashMap<>();
        this.motor = RobotTemplate.hardwareMap.get(DcMotor.class, motorName);
        this.motor.setDirection(stringToDirection(direction));
        setSpeed(1);
        setTolerance(5);
    }

    public void addPreset(Object key, int value) {presets.put(key, value);}
    public void removePreset(Object key) {presets.remove(key);}

    public void setSpeed(double speed) {this.speed = speed;}
    public double getSpeed() {return speed;}

    public void setMode(DcMotor.RunMode mode) {motor.setMode(mode);}
    public DcMotor.RunMode getMode() {return motor.getMode();}
    public void setPower(double power) {motor.setPower(power);}
    public double getPower() {return motor.getPower();}
    public void setTargetPosition(int position) {motor.setTargetPosition(position);}
    public int getTargetPosition() {return motor.getTargetPosition();}
    public int getCurrentPosition() {return motor.getCurrentPosition();}

    public void setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior setting) {
        motor.setZeroPowerBehavior(setting);
    }

    public DcMotor.ZeroPowerBehavior getZeroPowerBehaviour() {return motor.getZeroPowerBehavior();}

    public int getError() {return Math.abs(getCurrentPosition() - getTargetPosition());}

    public void setTolerance(int tolerance) {this.tolerance = tolerance;}
    public boolean isInPosition(int tolerance) {return getError() <= tolerance;}
    public boolean isInPosition() {return isInPosition(this.tolerance);}

    public void setPositionPreset(Object preset) {
        currentPreset = preset;
        setTargetPosition(presets.get(preset));
        setPower(speed);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setPositionEncoder(int value) {
        currentPreset = null;
        setTargetPosition(value);
        setPower(speed);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private DcMotorSimple.Direction stringToDirection(String input) {
        if (input.equalsIgnoreCase("F")) return DcMotorSimple.Direction.FORWARD;
        else return DcMotorSimple.Direction.REVERSE;
    }
}

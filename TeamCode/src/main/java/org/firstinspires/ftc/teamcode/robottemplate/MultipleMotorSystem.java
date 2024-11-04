package org.firstinspires.ftc.teamcode.robottemplate;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class MultipleMotorSystem implements RobotSubsystemTemplate {
    private DcMotor[] motors;
    private int tolerance;
    Map<Integer, Integer> offsets;

    public MultipleMotorSystem(DcMotor... motors) {
        offsets = new HashMap<>();
        this.motors = motors;
        setTolerance(5);
    }

    public MultipleMotorSystem(String... motorNames) {
        offsets = new HashMap<>();
        motors = new DcMotor[] {};
        ArrayList<DcMotor> tempList = new ArrayList<>();

        for (String motorName : motorNames) {
            tempList.add(RobotTemplate.hardwareMap.get(DcMotor.class, motorName));
        }

        motors = tempList.toArray(motors);
        setTolerance(5);
    }

    public void setMode(DcMotor.RunMode mode) {
        for (DcMotor actuator : motors) {actuator.setMode(mode);}
    }

    public DcMotor.RunMode getMode() {return motors[0].getMode();}

    public void setPower(double power) {
        for (DcMotor actuator : motors) {actuator.setPower(power);}
    }

    public double getPower() {return motors[0].getPower();}

    public void setTargetPosition(int value) {
        int index = 0;
        for (DcMotor actuator: motors) {
            actuator.setTargetPosition(value + offsets.getOrDefault(index, 0));
            index++;
        }
    }

    public int getTargetPosition() {
        return motors[0].getTargetPosition() - offsets.getOrDefault(0, 0);
    }

    public int getCurrentPosition() {
        return motors[0].getCurrentPosition() - offsets.getOrDefault(0, 0);
    }

    public void setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior setting) {
        for (DcMotor actuator : motors) {actuator.setZeroPowerBehavior(setting);}
    }

    public DcMotor.ZeroPowerBehavior getZeroPowerBehaviour() {
        return motors[0].getZeroPowerBehavior();
    }

    public int getError() {return Math.abs(getCurrentPosition() - getTargetPosition());}

    public void setTolerance(int tolerance) {this.tolerance = tolerance;}
    public boolean isInPosition(int tolerance) {return getError() <= tolerance;}
    public boolean isInPosition() {return isInPosition(this.tolerance);}

    public void setDirection(int index, DcMotorSimple.Direction direction) {
        motors[index].setDirection(direction);
    }

    public DcMotorSimple.Direction getDirection(int index) {return motors[index].getDirection();}

    public void forcePower(double power) {
        for (DcMotor actuator : motors) {
            actuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            actuator.setPower(power);
        }
    }

    public void quickReset() {
        DcMotor.RunMode oldMode = getMode();
        for (DcMotor actuator : motors) {
            actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            actuator.setMode(oldMode);
        }
    }

    public void setOffset(int index, int value) {offsets.put(index, value);}
}

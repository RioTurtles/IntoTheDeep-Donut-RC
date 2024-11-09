package org.firstinspires.ftc.teamcode.robottemplate;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class MultipleServoSystem {
    private ServoImplEx[] servos;
    Map<Integer, Double> offsets;
    
    public MultipleServoSystem(ServoImplEx... servos) {
        offsets = new HashMap<>();
        this.servos = servos;
    }

    public MultipleServoSystem(String... servoNames) {
        servos = new ServoImplEx[] {};
        offsets = new HashMap<>();
        ArrayList<ServoImplEx> tempList = new ArrayList<>();

        for (String motorName : servoNames) {
            tempList.add(RobotTemplate.hardwareMap.get(ServoImplEx.class, motorName));
        }

        servos = tempList.toArray(servos);
    }

    public void setDirection(int index, ServoImplEx.Direction direction) {
        servos[index].setDirection(direction);
    }

    public void setPwmEnable() {for (ServoImplEx servo : servos) {servo.setPwmEnable();}}
    public void setPwmDisable() {for (ServoImplEx servo : servos) {servo.setPwmDisable();}}
    public void setPwmRange(PwmControl.PwmRange range) {
        for (ServoImplEx servo : servos) {servo.setPwmRange(range);}
    }

    public void setPosition(double position) {
        int index = 0;
        for (ServoImplEx servo : servos) {
            servo.setPosition(position + offsets.getOrDefault(index, (double) 0));
            index++;
        }
    }

    public double getPosition() {
        return servos[0].getPosition() + offsets.getOrDefault(0, (double) 0);
    }

    public ServoImplEx.Direction getDirection(int index) {return servos[index].getDirection();}

    public void setOffsets(int index, double value) {offsets.put(index, value);}
}

package org.firstinspires.ftc.teamcode.robottemplate;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.ArrayList;

public class MultipleServoSystem {
    private ServoImplEx[] servos;
    
    public MultipleServoSystem(ServoImplEx... servos) {this.servos = servos;}
    public MultipleServoSystem(String... servoNames) {
        servos = new ServoImplEx[] {};
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
        for (ServoImplEx servo : servos) {servo.setPosition(position);}
    }

    public ServoImplEx.Direction getDirection(int index) {return servos[index].getDirection();}
}

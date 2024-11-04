package org.firstinspires.ftc.teamcode.robottemplate;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo.Direction;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.HashMap;
import java.util.Map;

public class SingleServoSystem implements RobotSubsystemTemplate {
    final private ServoImplEx servo;
    Map<Object, Double> presets;
    public Object currentPreset;

    public SingleServoSystem(ServoImplEx servo) {
        presets = new HashMap<>();
        this.servo = servo;
    }

    public SingleServoSystem(String servoName, String direction) {
        this.servo = RobotTemplate.hardwareMap.get(ServoImplEx.class, servoName);
        presets = new HashMap<>();
        servo.setDirection(stringToDirection(direction));
    }

    public void addPreset(Object key, double value) {presets.put(key, value);}
    public void removePreset(Object key) {presets.remove(key);}

    public void setPositionPreset(Object preset) {
        currentPreset = preset;
        setPosition(presets.get(preset));
    }

    public void setPosition(double position) {
        currentPreset = null;
        servo.setPosition(position);
    }

    private Direction stringToDirection(String input) {
        if (input.equalsIgnoreCase("F")) return Direction.FORWARD;
        else return Direction.REVERSE;
    }
}

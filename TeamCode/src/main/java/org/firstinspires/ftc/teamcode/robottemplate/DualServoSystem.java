package org.firstinspires.ftc.teamcode.robottemplate;

import com.qualcomm.robotcore.hardware.Servo.Direction;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.HashMap;
import java.util.Map;

public class DualServoSystem extends MultipleServoSystem implements RobotSubsystemTemplate {
    private ServoImplEx left, right;
    Map<Object, Double> presets;
    public Object currentPreset;

    public DualServoSystem(ServoImplEx left, ServoImplEx right) {
        super(left, right);
        presets = new HashMap<>();
    }

    public DualServoSystem(String nameLeft, String dL, String nameRight, String dR) {
        super(nameLeft, nameRight);
        presets = new HashMap<>();
        setDirection(0, stringToDirection(dL));
        setDirection(1, stringToDirection(dR));
    }

    public void addPreset(Object key, double value) {presets.put(key, value);}
    public void removePreset(Object key) {presets.remove(key);}

    public void setPositionPreset(Object preset) {
        currentPreset = preset;
        setPosition(presets.get(preset));
    }

    @Override
    public void setPosition(double position) {
        currentPreset = null;
        super.setPosition(position);
    }

    private Direction stringToDirection(String input) {
        if (input.equalsIgnoreCase("F")) return Direction.FORWARD;
        else return Direction.REVERSE;
    }
}

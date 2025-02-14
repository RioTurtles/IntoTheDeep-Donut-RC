package org.firstinspires.ftc.teamcode.robottemplate;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Servo.Direction;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

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

    public <E extends Enum<E>> void addPresets(@NonNull Class<E> enumeration) {
        for (@NonNull E constant : enumeration.getEnumConstants()) {
            // Get method via reflection
            Method method;
            try {method = constant.getClass().getMethod("get");}
            catch (NoSuchMethodException e) {
                throw new IllegalArgumentException("Cannot find getter methods for class " +
                        constant.getClass().getCanonicalName() + "; did you implement " +
                        GettableEnum.class.getName() + " ?");
            }

            // Run method and get its result, while catching exceptions
            double result;
            try {result = (double) Objects.requireNonNull(method.invoke(constant));}
            catch (IllegalAccessException | InvocationTargetException e) {
                throw new RuntimeException(e.getMessage());
            }

            addPreset(constant, result);
        }
    }

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

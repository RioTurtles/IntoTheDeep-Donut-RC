package org.firstinspires.ftc.teamcode.robottemplate;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

public class DualMotorSystem extends MultipleMotorSystem implements RobotSubsystemTemplate {
    final private DcMotor left, right;
    private double speed;
    Map<Object, Integer> presets;
    public Object currentPreset = null;

    public DualMotorSystem(DcMotor left, DcMotor right) {
        super(left, right);
        presets = new HashMap<>();
        this.left = left; this.right = right;
        setSpeed(1);
    }

    public DualMotorSystem(String nameLeft, String dLeft, String nameRight, String dRight) {
        super(nameLeft, nameRight);
        presets = new HashMap<>();
        setDirection(0, stringToDirection(dLeft));
        setDirection(1, stringToDirection(dRight));
        this.left = RobotTemplate.hardwareMap.get(DcMotor.class, nameLeft);
        this.right = RobotTemplate.hardwareMap.get(DcMotor.class, nameRight);
        this.left.setDirection(stringToDirection(dLeft));
        this.right.setDirection(stringToDirection(dRight));
        setSpeed(1);
    }

    public void addPreset(Object key, int value) {presets.put(key, value);}
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
            int result;
            try {result = (int) Objects.requireNonNull(method.invoke(constant));}
            catch (IllegalAccessException | InvocationTargetException e) {
                throw new RuntimeException(e.getMessage());
            }

            addPreset(constant, result);
        }
    }

    public void setSpeed(double speed) {this.speed = speed;}
    public double getSpeed() {return speed;}

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

    private Direction stringToDirection(String input) {
        if (input.equalsIgnoreCase("F")) return Direction.FORWARD;
        else return Direction.REVERSE;
    }

    @Override
    public void setOffset(int left, int right) {
        super.setOffset(0, left);
        super.setOffset(1, right);
    }
}

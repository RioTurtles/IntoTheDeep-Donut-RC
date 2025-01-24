package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


/**
 * This class represents a helper for Roadrunner-like pathing. This simulates pathing to
 * specific coordinates through the help of PID controllers and pose estimates through Roadrunner.
 * To use this helper, Roadrunner only needs to be tuned up to LocalisationTest. Ensure that
 * coordinates are accurate before using this class.
 */
public class PartialRoadrunnerHelper {
    private final SampleMecanumDrive drive;
    private final JoystickMethod remote;
    private final PIDCoefficients pidX, pidY, pidH;  // Heading variables are in RADIANS
    private double targetX, targetY, targetH;
    private double toleranceX, toleranceY, toleranceH;

    private double integral1, integral2, integral3;
    private double lastError1, lastError2, lastError3;

    public PartialRoadrunnerHelper(SampleMecanumDrive drivetrain, JoystickMethod method) {
        drive = drivetrain;
        remote = method;
        pidX = new PIDCoefficients(0.05, 0, 0);
        pidY = new PIDCoefficients(0.05, 0, 0);
        pidH = new PIDCoefficients(0.6, 0, 0);
        targetX = getX();
        targetY = getY();
        targetH = getHeadingDegrees();
        toleranceX = 0.4;
        toleranceY = 0.4;
        toleranceH = Math.toRadians(1.00);
    }

    public void setPIDCoefficients(Axis axis, double kP, double kI, double kD) {
        switch (axis) {
            case X: pidX.copyFrom(new PIDCoefficients(kP, kI, kD)); break;
            case Y: pidY.copyFrom(new PIDCoefficients(kP, kI, kD)); break;
            case HEADING: pidH.copyFrom(new PIDCoefficients(kP, kI, kD)); break;
        }
    }

    public PIDCoefficients getPIDCoefficients(Axis axis) {
        switch (axis) {
            case X: return pidX;
            case Y: return pidY;
            case HEADING: return pidH;
            default: return null;
        }
    }

    public void setToleranceX(double x) {toleranceX = x;}
    public void setToleranceY(double y) {toleranceY = y;}
    public void setToleranceHeading(double degrees) {toleranceH = Math.toRadians(degrees);}

    public double getToleranceX() {return toleranceX;}
    public double getToleranceY() {return toleranceY;}
    public double getToleranceHeadingDegrees() {return Math.toDegrees(toleranceH);}
    public double getToleranceHeadingRadians() {return toleranceH;}

    public void setPoseEstimate(Pose2d poseEstimate) {drive.setPoseEstimate(poseEstimate);}
    public void setPoseEstimate(double x, double y, double headingDegrees) {
        drive.setPoseEstimate(new Pose2d(x, y, Math.toRadians(headingDegrees)));
    }

    public Pose2d getPoseEstimate() {return drive.getPoseEstimate();}
    public double getX() {return getPoseEstimate().getX();}
    public double getY() {return getPoseEstimate().getY();}
    public double getHeadingDegrees() {return Math.toDegrees(getPoseEstimate().getHeading());}
    public double getHeadingRadians() {return getPoseEstimate().getHeading();}

    public void setTargetX(double x) {targetX = x;}
    public void setTargetY(double y) {targetY = y;}
    public void setTargetHeading(double degrees) {targetH = Math.toRadians(degrees);}

    public void setTarget(double x, double y) {setTargetX(x); setTargetY(y);}
    public void setTarget(double x, double y, double headingDegrees) {
        setTarget(x, y);
        setTargetHeading(headingDegrees);
    }

    public void setTarget(Pose2d pose) {
        setTarget(pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
    }

    public Pose2d getTarget() {return new Pose2d(targetX, targetY, Math.toRadians(targetH));}
    public double getTargetX() {return getTarget().getX();}
    public double getTargetY() {return getTarget().getY();}
    public double getTargetHeadingDegrees() {return Math.toDegrees(getTarget().getHeading());}
    public double getTargetHeadingRadians() {return getTarget().getHeading();}

    public double getErrorX() {return targetX - getX();}
    public double getErrorY() {return targetY - getY();}
    public double getErrorHeadingDegrees() {return Math.toDegrees(targetH) - getHeadingDegrees();}
    public double getErrorHeadingRadians() {return targetH - getHeadingRadians();}

    public double getAbsErrorX() {return Math.abs(getErrorX());}
    public double getAbsErrorY() {return Math.abs(getErrorY());}
    public double getAbsErrorHeadingDegrees() {return Math.abs(getErrorHeadingDegrees());}
    public double getAbsErrorHeadingRadians() {return Math.abs(getErrorHeadingRadians());}

    public boolean isInPosition() {
        return (getAbsErrorX() <= toleranceX)
                && (getAbsErrorY() <= toleranceY)
                && (getAbsErrorHeadingRadians() <= toleranceH);
    }

    public void update() {
        double botHeading = getHeadingRadians();
        double e1 = (targetX - getX());
        double leftY = ((e1) * pidY.kP + integral1 * pidY.kI + ((e1 - lastError1) * pidY.kD));
        double e2 = (getY() - targetY);
        double leftX = -((e2) * pidX.kP + integral2 * pidX.kI +((e2 - lastError2) * pidX.kD));
        double e3 = (targetH - botHeading);

        if (e3 > Math.PI) e3 -= 2 * Math.PI;
        if (e3 < -Math.PI) e3 += 2 * Math.PI;
        double rx = -((e3) * pidH.kP + integral3 * pidH.kI +((e3 - lastError3) * pidH.kD));
        if (Math.abs(e3) < toleranceH) rx = 0;

        integral1 += e1;
        integral2 += e2;
        integral3 += e3;
        lastError1 = e1;
        lastError2 = e2;
        lastError3 = e3;

        if ((Math.abs(leftX) > 0.5 || Math.abs(leftY) > 0.5) && leftX != 0 && leftY != 0) {
            if (Math.abs(leftY) > Math.abs(leftX)) {
                leftX *= Math.abs(0.5 / leftY);
                if (leftY > 0) leftY = 0.5; else leftY = -0.5;
            } else {
                leftY *= Math.abs(0.5 / leftX);
                if (leftX > 0) leftX = 0.5; else leftX = -0.5;
            }
        }

        remote.call(leftY, -leftX, rx, botHeading);
        drive.update();
    }

    public String toTelemetry() {
        return "X: " + Math.round(getX() * 100) / 100 + " [E: "
                + Math.round(getErrorX() * 100) / 100 + " | T: " + getTargetX() + "]\n"
                + "Y: " + Math.round(getY() * 100) / 100 + " [E: "
                + Math.round(getErrorY() * 100) / 100 + " | T: " + getTargetY() + "]\n"
                + "H(D): " + Math.round(getHeadingDegrees() * 100) / 100 + " [E: "
                + Math.round(getErrorHeadingDegrees() * 100) / 100 + " | T: "
                + getTargetHeadingDegrees() + "]\n" + "H(R): "
                + Math.round(getHeadingRadians() * 100) / 100 + " [E: "
                + Math.round(getErrorHeadingRadians() * 100) / 100
                + " | T: " + Math.toRadians(getTargetHeadingRadians()) + "]\n";
    }
}


class PIDCoefficients {
    public double kP, kI, kD;

    public PIDCoefficients(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void copyFrom(@NonNull PIDCoefficients source) {
        kP = source.kP;
        kI = source.kI;
        kD = source.kD;
    }

    public void setP(double p) {kP = p;}
    public void setI(double i) {kI = i;}
    public void setD(double d) {kD = d;}
}


interface JoystickMethod {void call(double v, double h, double pivot, double heading);}

enum Axis {X, Y, HEADING}

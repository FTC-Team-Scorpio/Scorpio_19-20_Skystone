package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class MotorBlock {
    Motor left1;
    Motor right1;
    Motor left2;
    Motor right2;
    int leftmodifier = -1;
    int rightmodifier = 1;
    double multiplier = 0.5;
    BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    Orientation lastAngles = new Orientation();
    double                  globalAngle;
    HardwareMap hardwareMap2;
    Telemetry telemetry2;
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    public MotorBlock (Motor left1, Motor right1, Motor left2, Motor right2) {
        this.left1 = left1;
        this.right1 = right1;
        this.left2 = left2;
        this.right2 = right2;
    }
    public void forward (double speed) {
        left1.setPower(speed * leftmodifier);
        left2.setPower(speed * leftmodifier);
        right1.setPower(speed * rightmodifier);
        right2.setPower(speed * rightmodifier);
    }
    public void forwardrotations (double speed, double distance) {
        double rotations = distance / 12.5663706144;
        int ticks = (int) ((rotations * 1120) * multiplier);
        int leftticks = ticks * leftmodifier;
        int rightticks = ticks * rightmodifier;
        left1.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left2.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right1.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right2.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left1.motor.setTargetPosition(leftticks);
        left2.motor.setTargetPosition(leftticks);
        right1.motor.setTargetPosition(-rightticks);
        right2.motor.setTargetPosition(-rightticks);
        this.forward(speed/1.5);
        left1.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left2.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right1.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right2.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (left1.motor.isBusy() && right1.motor.isBusy() && left2.motor.isBusy() && right2.motor.isBusy()) {
            //wait
        }
        this.stop();
    }
    public void backward (double speed) {
        this.forward(-speed);
    }
    public void backwardrotations (double speed, double distance) {
        double rotations = distance / 12.5663706144;
        int ticks = (int) ((rotations * 1120) * multiplier);
        int leftticks = ticks * leftmodifier * -1;
        int rightticks = ticks * rightmodifier * -1;
        left1.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left2.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right1.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right2.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left1.motor.setTargetPosition(-leftticks);
        left2.motor.setTargetPosition(-leftticks);
        right1.motor.setTargetPosition(rightticks);
        right2.motor.setTargetPosition(rightticks);
        this.backward(speed/1.5);
        left1.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left2.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right1.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right2.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (left1.motor.isBusy() && right1.motor.isBusy() && left2.motor.isBusy() && right2.motor.isBusy()) {
            //wait
        }
        this.stop();
    }
    public void left (double speed) {
        left1.setPower(speed * -1 * leftmodifier);
        left2.setPower(speed * -1 * leftmodifier);
        right1.setPower(speed * rightmodifier);
        right2.setPower(speed * rightmodifier);
    }
    public void leftcurve (double speed) {
        right1.setPower(speed * rightmodifier);
        right2.setPower(speed * rightmodifier);
        left1.setPower(speed * -1 * leftmodifier);
        left2.setPower(speed * leftmodifier);
    }
    public void leftcurve2 (double speed) {
        left1.setPower(speed * -1 * leftmodifier);
        left2.setPower(speed * -1 * leftmodifier);
    }
    public void right (double speed) {
        left1.setPower(speed * leftmodifier);
        left2.setPower(speed * leftmodifier);
        right1.setPower(speed * -1 * rightmodifier);
        right2.setPower(speed * -1 * rightmodifier);
    }
    public void rightcurve (double speed) {
        left1.setPower(speed * leftmodifier);
        left2.setPower(speed * leftmodifier);
        right2.setPower(speed * rightmodifier);
        right1.setPower(speed * -1 * rightmodifier);
    }
    public void rightcurve2 (double speed) {
        right1.setPower(speed * -1 * rightmodifier);
        right2.setPower(speed * -1 * rightmodifier);
    }
    public void stop () {
        left1.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left1.stop();
        left2.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left2.stop();
        right1.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right1.stop();
        right2.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right2.stop();
    }
    public void leftsideways (double speed) {
        left1.setPower(-speed * leftmodifier);
        left2.setPower(speed * leftmodifier);
        right1.setPower(speed * rightmodifier);
        right2.setPower(-speed * rightmodifier);
    }
    public void leftsidewaysrotations (double speed, double distance, BNO055IMU emu) {
        imu = emu;
        double rotations = distance / 12.5663706144;
        int ticks = (int) (rotations * 1120);
        int leftticks = ticks * leftmodifier;
        int rightticks = ticks * rightmodifier;
        left1.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left2.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right1.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right2.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left1.motor.setTargetPosition(leftticks);
        left2.motor.setTargetPosition(leftticks);
        right1.motor.setTargetPosition(-rightticks);
        right2.motor.setTargetPosition(-rightticks);
        this.leftsideways(speed);
        left1.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left2.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right1.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right2.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        resetAngle();
        while (left1.motor.isBusy() && right1.motor.isBusy() && left2.motor.isBusy() && right2.motor.isBusy()) {
            /*if (getAngle() > 0) {
                left1.setPower(-speed * leftmodifier * 0.9);
                left2.setPower(speed * leftmodifier);
                right1.setPower(speed * rightmodifier);
                right2.setPower(-speed * rightmodifier * 0.9);
            }
            else if (getAngle() < 0) {
                left1.setPower(-speed * leftmodifier);
                left2.setPower(speed * leftmodifier * 0.9);
                right1.setPower(speed * rightmodifier * 0.9);
                right2.setPower(-speed * rightmodifier);
            }
            else {
                left1.setPower(-speed * leftmodifier);
                left2.setPower(speed * leftmodifier);
                right1.setPower(speed * rightmodifier);
                right2.setPower(-speed * rightmodifier);
            }*/
        }
        this.stop();
    }
    public void leftsidewaysrotations (double speed, double distance) {
        double rotations = distance / 12.5663706144;
        int ticks = (int) (rotations * 1120);
        int leftticks = ticks * leftmodifier;
        int rightticks = ticks * rightmodifier;
        left1.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left2.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right1.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right2.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left1.motor.setTargetPosition(leftticks);
        left2.motor.setTargetPosition(leftticks);
        right1.motor.setTargetPosition(-rightticks);
        right2.motor.setTargetPosition(-rightticks);
        this.leftsideways(speed);
        left1.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left2.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right1.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right2.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        resetAngle();
        while (left1.motor.isBusy() && right1.motor.isBusy() && left2.motor.isBusy() && right2.motor.isBusy()) {
            /*if (getAngle() > 0) {
                left1.setPower(-speed * leftmodifier * 0.9);
                left2.setPower(speed * leftmodifier);
                right1.setPower(speed * rightmodifier);
                right2.setPower(-speed * rightmodifier * 0.9);
            }
            else if (getAngle() < 0) {
                left1.setPower(-speed * leftmodifier);
                left2.setPower(speed * leftmodifier * 0.9);
                right1.setPower(speed * rightmodifier * 0.9);
                right2.setPower(-speed * rightmodifier);
            }
            else {
                left1.setPower(-speed * leftmodifier);
                left2.setPower(speed * leftmodifier);
                right1.setPower(speed * rightmodifier);
                right2.setPower(-speed * rightmodifier);
            }*/
        }
        this.stop();
    }
    // Mecanum wheels ONLY
    public void rightsideways (double speed) {
        left1.setPower(speed * leftmodifier);
        left2.setPower(-speed * leftmodifier);
        right1.setPower(-speed * rightmodifier);
        right2.setPower(speed * rightmodifier);
    }
    public void rightsidewaysrotations (double speed, double distance) {
        double rotations = distance / 12.5663706144;
        int ticks = (int) (rotations * 1120);
        int leftticks = ticks * leftmodifier;
        int rightticks = ticks * rightmodifier;
        left1.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left2.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right1.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right2.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left1.motor.setTargetPosition(leftticks);
        left2.motor.setTargetPosition(leftticks);
        right1.motor.setTargetPosition(-rightticks);
        right2.motor.setTargetPosition(-rightticks);
        this.rightsideways(speed);
        left1.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left2.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right1.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right2.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (left1.motor.isBusy() && right1.motor.isBusy() && left2.motor.isBusy() && right2.motor.isBusy()) {
            //wait
        }
        this.stop();
    }
    public void frontright (double speed) {
        left1.setPower(speed * leftmodifier);
        right2.setPower(speed * rightmodifier);
    }
    /*public void frontright (double speed, double distance) {
        left1.setPower(speed * leftmodifier);
        right2.setPower(speed * rightmodifier);
        left1.waitfordistance(distance * leftmodifier);
    }*/
    public void backright (double speed) {
        left2.setPower(-speed * leftmodifier);
        right1.setPower(-speed * rightmodifier);
    }
    /*public void backright (double speed, double distance) {
        left2.setPower(-speed * leftmodifier);
        right1.setPower(-speed * rightmodifier);
        left2.waitfordistance(-distance * leftmodifier);
    }*/
    public void frontleft (double speed) {
        left2.setPower(speed * leftmodifier);
        right1.setPower(speed * rightmodifier);
    }
    /*public void frontleft (double speed, double distance) {
        left2.setPower(speed * leftmodifier);
        right1.setPower(speed * rightmodifier);
        left2.waitfordistance(distance * leftmodifier);
    }*/
    public void backleft (double speed) {
        left1.setPower(-speed * leftmodifier);
        right2.setPower(-speed * rightmodifier);
    }
    /*public void backleft (double speed, double distance) {
        left1.setPower(-speed * leftmodifier);
        right2.setPower(-speed * rightmodifier);
        left1.waitfordistance(-speed * leftmodifier);
    }*/
    public void tank (double leftspeed, double rightspeed) {
        left1.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left2.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right2.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right1.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left1.setPower(leftspeed * leftmodifier);
        left2.setPower(leftspeed * leftmodifier);
        right2.setPower(rightspeed * rightmodifier);
        right1.setPower(rightspeed * rightmodifier);
    }
}

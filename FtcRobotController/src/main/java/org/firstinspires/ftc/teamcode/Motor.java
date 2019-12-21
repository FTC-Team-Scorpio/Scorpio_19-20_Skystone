package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

public class Motor {
    DcMotor motor;
    double CIRCUMFERENCE = 12.566;
    public void setPower(double power) {
        motor.setPower(power);
    }
    public void stop() {
        motor.setPower(0.0);
    }
    public Motor (DcMotor motor) {
        this.motor = motor;
    }
}

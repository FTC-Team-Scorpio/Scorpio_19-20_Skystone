package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class skystone extends LinearOpMode {
    Motor left1;
    Motor right1;
    Motor left2;
    Motor right2;
    MotorBlock block;
    Servo deliverer;
    public void runOpMode () {
        left1 = new Motor(hardwareMap.get(DcMotor.class, "left1"));
        right1 = new Motor(hardwareMap.get(DcMotor.class, "right1"));
        left2 = new Motor(hardwareMap.get(DcMotor.class, "left2"));
        right2 = new Motor(hardwareMap.get(DcMotor.class, "right2"));
        block = new MotorBlock(left1, right1, left2, right2);
        deliverer = hardwareMap.get(Servo.class,"deliverer");

        waitForStart();
        block.leftsideways((0.3+1)/2);
        sleep(2900);
        deliverer.setPosition(1);
        block.rightsideways((0.3+1)/2);
        sleep(2900);
        block.backwardrotations(1,20);
        deliverer.setPosition(0);
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class park extends LinearOpMode {
    Motor right1;
    Motor left1;
    Motor left2;
    Motor right2;
    MotorBlock block;
    CRServo foldservo1;
    CRServo foldservo2;
    CRServo horizlinear;
    public void runOpMode () {
        left1 = new Motor(hardwareMap.get(DcMotor.class, "left1"));
        right1 = new Motor(hardwareMap.get(DcMotor.class, "right1"));
        left2 = new Motor(hardwareMap.get(DcMotor.class, "left2"));
        right2 = new Motor(hardwareMap.get(DcMotor.class, "right2"));
        horizlinear = hardwareMap.get(CRServo.class,"horizlinear");
        block = new MotorBlock(left1,right1,left2,right2);
        foldservo1 = hardwareMap.get(CRServo.class,"foldservo1");
        foldservo2 = hardwareMap.get(CRServo.class,"foldservo2");
        waitForStart();

        foldservo1.setPower(-1);
        foldservo2.setPower(1);
        sleep(2000);

        block.forwardrotations(1,12);
    }

}

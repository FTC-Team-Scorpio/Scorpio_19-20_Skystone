package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
@Disabled
public class RIGHT_TRIANGLE_CURVETURN extends LinearOpMode {
    Motor left1;
    Motor right1;
    Motor left2;
    Motor right2;
    Motor intake1;
    Motor intake2;
    MotorBlock block;
    CRServo foldservo1;
    CRServo foldservo2;
    double frontspeed = 0.6;
    double turnspeed = 0.5;
    public void runOpMode() throws InterruptedException {
        //Variable Definitions
        left1 = new Motor(hardwareMap.get(DcMotor.class, "left1"));
        right1 = new Motor(hardwareMap.get(DcMotor.class, "right1"));
        left2 = new Motor(hardwareMap.get(DcMotor.class, "left2"));
        right2 = new Motor(hardwareMap.get(DcMotor.class, "right2"));
        intake1 = new Motor(hardwareMap.get(DcMotor.class,"intake1"));
        intake2 = new Motor(hardwareMap.get(DcMotor.class,"intake2"));
        foldservo1 = hardwareMap.get(CRServo.class,"foldservo1");
        foldservo2 = hardwareMap.get(CRServo.class,"foldservo2");
        block = new MotorBlock(left1, right1, left2, right2);
        Servo mover1 = hardwareMap.get(Servo.class, "mover1");
        Servo mover2 = hardwareMap.get(Servo.class, "mover2");

        waitForStart();

        foldservo1.setPower(1);
        foldservo2.setPower(-1);

        block.rightsidewaysrotations(0.25,12,false);

        block.backwardrotations(frontspeed, 34);

        block.stop();

        mover1.setPosition(1);
        mover2.setPosition(0);

        sleep(1000);

        /*block.frontbla(frontspeed);
        sleep(1500);*/

        block.left(0.425);
        sleep(500);

        block.forwardrotations(frontspeed, 20);

        /*block.right(turnspeed);
        sleep(500);*/

        block.tank(0,0.5);
        sleep(5700);

        block.stop();

        mover1.setPosition(0);
        mover2.setPosition(1);

        //intake1.setPower(0.5);
        //intake2.setPower(0.5);

        block.backwardrotationstimer(1,20);

        intake1.stop();
        intake2.stop();

        foldservo1.setPower(-1);
        foldservo2.setPower(1);

        block.leftsidewaysrotations(1,5);

        block.forwardrotations(1,30);

        block.stop();
    }
}

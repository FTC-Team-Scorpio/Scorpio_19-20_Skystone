package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class LEFT_FOUNDATION extends LinearOpMode {
    Motor left1;
    Motor right1;
    Motor left2;
    Motor right2;
    MotorBlock block;
    double frontspeed = 0.4;
    double turnspeed = 0.1;
    public void runOpMode() throws InterruptedException {
        //Variable Definitions
        left1 = new Motor(hardwareMap.get(DcMotor.class, "left1"));
        right1 = new Motor(hardwareMap.get(DcMotor.class, "right1"));
        left2 = new Motor(hardwareMap.get(DcMotor.class, "left2"));
        right2 = new Motor(hardwareMap.get(DcMotor.class, "right2"));
        block = new MotorBlock(left1,right1,left2,right2);
        Servo mover1 = hardwareMap.get(Servo.class,"mover1");
        Servo mover2 = hardwareMap.get(Servo.class,"mover2");

        waitForStart();

        block.backwardrotations(frontspeed,34);

        block.stop();

        mover1.setPosition(1);
        mover2.setPosition(1);

        sleep(300);

        /*block.frontbla(frontspeed);
        sleep(1500);*/

        block.forwardrotations(frontspeed,35);

        /*block.left(turnspeed);
        sleep(500);*/

        block.stop();

        sleep(1000);

        mover1.setPosition(0);
        mover2.setPosition(0);

        block.rightsidewaysrotations(0.7,30);
        block.stop();
        sleep(200);

        block.backwardrotations(frontspeed,16);
        block.stop();
        sleep(200);

        block.leftsidewaysrotations(1,26);
        block.stop();
        sleep(200);


        Motor turnclaw = new Motor(hardwareMap.get(DcMotor.class, "turnclaw"));
        turnclaw.setPower(0.5);
        sleep(1000);
        turnclaw.setPower(0);

        /*block.backwardrotations(frontspeed,2);
        block.stop();
        sleep(200);*/

        block.rightsidewaysrotations(1,30);
        block.stop();

        block.stop();

    }
}


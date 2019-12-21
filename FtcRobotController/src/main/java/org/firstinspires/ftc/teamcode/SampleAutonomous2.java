package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class SampleAutonomous2 extends LinearOpMode {
    Motor left1;
    Motor right1;
    Motor left2;
    Motor right2;
    MotorBlock block;
    double frontspeed = 0.25;
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

        block.backward(frontspeed);
        sleep(2600);

        block.stop();

        mover1.setPosition(1);
        mover2.setPosition(1);

        sleep(300);

        /*block.frontright(frontspeed);
        sleep(1500);*/

        block.forward(frontspeed);
        sleep(5000);

        /*block.left(turnspeed);
        sleep(500);*/

        block.stop();

        sleep(1000);

        mover1.setPosition(0);
        mover2.setPosition(0);

        block.rightsideways(1);

        sleep(1400);

        block.backward(frontspeed);
        sleep(1200);

        block.leftsideways(1);
        sleep(900);
        block.stop();

        Motor turnclaw = new Motor(hardwareMap.get(DcMotor.class, "turnclaw"));
        turnclaw.setPower(0.25);
        sleep(1500);
        turnclaw.setPower(0);

        block.forward(frontspeed);
        sleep(1000);

        block.rightsideways(1);
        sleep(1500);
    }
}


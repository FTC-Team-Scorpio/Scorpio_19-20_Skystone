package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class park extends LinearOpMode {
    Motor right1;
    Motor left1;
    Motor left2;
    Motor right2;
    MotorBlock block;
    public void runOpMode () {
        right1 = new Motor(hardwareMap.get(DcMotor.class,"right1"));
        left1 = new Motor(hardwareMap.get(DcMotor.class,"left1"));
        right2 = new Motor(hardwareMap.get(DcMotor.class,"right2"));
        left2 = new Motor(hardwareMap.get(DcMotor.class,"left2"));
        block = new MotorBlock(left1,right1,left2,right2);
        waitForStart();
        block.forwardrotations(1,12);
        block.stop();
    }

}

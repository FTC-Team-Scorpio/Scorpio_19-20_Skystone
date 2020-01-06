package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class right extends LinearOpMode {
    Motor left1;
    Motor right1;
    Motor left2;
    Motor right2;
    MotorBlock block;
    public void runOpMode () {
        left1 = new Motor(hardwareMap.get(DcMotor.class, "left1"));
        right1 = new Motor(hardwareMap.get(DcMotor.class, "right1"));
        left2 = new Motor(hardwareMap.get(DcMotor.class, "left2"));
        right2 = new Motor(hardwareMap.get(DcMotor.class, "right2"));
        block = new MotorBlock(left1,right1,left2,right2);

        waitForStart();

        block.rightsidewaysrotations(0.5,2);
    }
}

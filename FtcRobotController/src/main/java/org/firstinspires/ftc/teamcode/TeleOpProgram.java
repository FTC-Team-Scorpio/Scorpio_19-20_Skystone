package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp /*Tele Op*/
public class TeleOpProgram extends LinearOpMode {
    //Variable Definitions
    Motor left1;
    Motor right1;
    Motor left2;
    Motor right2;
    Motor linear1;
    Motor linear2;
    Motor intake1;
    Motor intake2;
    MotorBlock block;
    Servo claw1;
    Servo claw2;
    Servo mover1;
    Servo mover2;
    Motor lifter1;
    Motor lifter2;
    double frontspeed = 1;
    double turnspeed = 0.6;
    double sidewaysspeed = 1;
    //Main function
    public void runOpMode() {
        //Variable Definitions
        left1 = new Motor(hardwareMap.get(DcMotor.class, "left1"));
        right1 = new Motor(hardwareMap.get(DcMotor.class, "right1"));
        left2 = new Motor(hardwareMap.get(DcMotor.class, "left2"));
        right2 = new Motor(hardwareMap.get(DcMotor.class, "right2"));
        linear1 = new Motor(hardwareMap.get(DcMotor.class, "linear1"));
        linear2 = new Motor(hardwareMap.get(DcMotor.class,"linear2"));
        intake1 = new Motor(hardwareMap.get(DcMotor.class,"intake1"));
        intake2 = new Motor(hardwareMap.get(DcMotor.class,"intake2"));
        block = new MotorBlock(left1,right1,left2,right2);
        claw1 = hardwareMap.get(Servo.class,"claw1");
        claw2 = hardwareMap.get(Servo.class,"claw2");
        mover1 = hardwareMap.get(Servo.class,"mover1");
        mover2 = hardwareMap.get(Servo.class,"mover2");//While OpMode is running
        waitForStart();
        while (opModeIsActive()) {
            drivetrain(); /*drivetrain functions*/
            slowmode(); /*activate/deactivate slowmode*/
            //lifter();
            //turnclaw();
            //claw();
            mover();
            linearslide();
            intake();
        }
    }
    double speed = 1;
    public void intake () {
        if (gamepad2.dpad_up) speed = 0.6;
        if (gamepad2.dpad_left) speed = 0.75;
        if (gamepad2.dpad_down) speed = 1;
        if (gamepad2.right_trigger != 0) {
            intake1.setPower(-speed);
            intake2.setPower(speed);
        }
        else if (gamepad2.left_trigger != 0) {
            intake1.setPower(speed);
            intake2.setPower(-speed);
        }
        else {
            intake1.setPower(0);
            intake2.setPower(0);
        }
    }
    public void slowmode () {
        if (gamepad1.dpad_up) {
            frontspeed = 0.25;
            sidewaysspeed = 0.3;
            turnspeed = 0.25;
            telemetry.addData("mode","slow");

        }
        if (gamepad1.dpad_down) {
            frontspeed = 1;
            turnspeed = 0.6;
            sidewaysspeed = 1;
            telemetry.addData("mode","normal");
        }
        if (gamepad1.dpad_left) {
            frontspeed = (0.25+1)/2;
            sidewaysspeed = (0.3+1)/2;
            turnspeed = (0.25+0.6)/2;
            telemetry.addData("mode","medium");
        }
        telemetry.update();
    }
    //Manages drivetrain functions
    public void drivetrain () {
        //Forward (if left stick forward)
        if (gamepad1.left_stick_y < 0) {
            block.forward(frontspeed);
        }
        //Backward (if left stick backward)
        else if (gamepad1.left_stick_y > 0) {
            block.backward(frontspeed);
        }
        //Left (if right stick left)
        else if (gamepad1.right_stick_x < 0) {
            block.left(turnspeed);
        }
        //Right (if right stick right)
        else if (gamepad1.right_stick_x > 0) {
            block.right(turnspeed);
        }
        //Sideways Left (if left trigger pressed)
        else if (gamepad1.left_trigger > 0) {
            block.leftsideways(sidewaysspeed);
        }
        //Sideways Right (if right trigger pressed)
        else if (gamepad1.right_trigger > 0) {
            block.rightsideways(sidewaysspeed);
        }
        //Diagonal Right --> works
        else if (gamepad1.b && gamepad1.y) {
            block.frontright(frontspeed);
        }
        //works
        else if (gamepad1.x && gamepad1.y) {
            block.frontleft(frontspeed);
        }
        else if (gamepad1.a && gamepad1.b) {
            block.backright(frontspeed);
        }
        else if (gamepad1.x && gamepad1.a) {
            block.backleft(frontspeed);
        }
        //Stop (if no moving triggers happen)
        else {
            block.left2.stop();
            block.left1.stop();
            block.right1.stop();
            block.right2.stop();
        }
    }
    public void claw () {
        if (gamepad2.left_trigger != 0) {
            claw1.setPosition(1);
            claw2.setPosition(1);
        }
        if (gamepad2.right_trigger != 0) {
            claw1.setPosition(0);
            claw2.setPosition(0);
        }
    }
    public void mover () {
        if (gamepad2.a) {
            mover1.setPosition(1);
            mover2.setPosition(1);

        }
        if (gamepad2.y) {
            mover1.setPosition(0);
            mover2.setPosition(0);
        }
    }

    public void linearslide () {
        //If gamepad 2 left stick pointing up
        if (gamepad2.left_stick_y < 0) {
            //Move linear slide up
            linear1.setPower(0.5);
            linear2.setPower(-0.5);
        }
        //If gamepad 2 left stick pointing up
        else if (gamepad2.left_stick_y > 0) {
            //Move linear slide down
            linear1.setPower(-0.3);
            linear2.setPower(0.3);
        }
        //If gamepad 2 left stick in center
        else {
            //Stop linear slide
            linear1.setPower(0);
            linear2.setPower(0);
        }
    }
    public void turnclaw () {
        DcMotor turnclaw = hardwareMap.get(DcMotor.class,"turnclaw");
        //If gamepad 2 left stick pointing up
        if (gamepad2.right_stick_y < 0) {
            //Move linear slide up
            turnclaw.setPower(-0.25);
        }
        //If gamepad 2 left stick pointing up
        else if (gamepad2.right_stick_y > 0) {
            //Move linear slide down
            turnclaw.setPower(0.25);
        }
        //If gamepad 2 left stick in center
        else {
            //Stop linear slide
            turnclaw.setPower(0);
        }
    }

}

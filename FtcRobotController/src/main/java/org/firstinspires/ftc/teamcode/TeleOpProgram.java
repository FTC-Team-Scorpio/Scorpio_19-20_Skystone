package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

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
    Servo claw;
    Servo mover1;
    Servo mover2;
    Motor lifter1;
    Motor lifter2;
    Servo deliverer;
    CRServo horizlinear;
    CRServo foldservo1;
    CRServo foldservo2;
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
        deliverer = hardwareMap.get(Servo.class,"deliverer");
        horizlinear = hardwareMap.get(CRServo.class,"horizlinear");
        block = new MotorBlock(left1,right1,left2,right2);
        claw = hardwareMap.get(Servo.class,"claw");
        mover1 = hardwareMap.get(Servo.class,"mover1");
        mover2 = hardwareMap.get(Servo.class,"mover2");//While OpMode is running
        telemetry.addData("bla",horizlinear.getPower());
        waitForStart();
        while (opModeIsActive()) {
            drivetrain(); /*drivetrain functions*/
            slowmode(); /*activate/deactivate slowmode*/
            claw(); //claw
            horizontallinear(); //horizontal linear slide
            mover(); //foundation mover
            linearslide(); //verticle linear slide
            intake(); //intake mechanism
            delivery(); //delivery mechanism
        }
    }
    public void horizontallinear () {
        if (gamepad2.right_stick_y > 0) {
            horizlinear.setPower(1);
        }
        else if (gamepad2.right_stick_y < 0) {
            horizlinear.setPower(-1);
        }
        else {
            horizlinear.setPower(0.01);
        }
    }
    public void claw () {
        if (gamepad2.left_trigger != 0) {
            claw.setPosition(0);
        }
        else if (gamepad2.right_trigger != 0) {
            claw.setPosition(1);
        }
    }
    boolean wasright = false;
    int deliverypos = -1;
    public void delivery () {
        if (gamepad2.dpad_right && !wasright) {
            wasright = true;
            deliverypos *= -1;
            deliverer.setPosition(Range.scale(deliverypos,-1,1,0,1));
        }
        else {
            wasright = false;
        }
    }
    double intakespeed = 1;
    public void intake () {
        if (gamepad2.dpad_up) intakespeed = 0.5;
        if (gamepad2.dpad_left) intakespeed = 0.75;
        if (gamepad2.dpad_down) intakespeed = 1;
        if (gamepad2.right_bumper) {
            intake1.setPower(-intakespeed);
            intake2.setPower(intakespeed);
        }
        else if (gamepad2.left_bumper) {
            intake1.setPower(intakespeed);
            intake2.setPower(-intakespeed);
        }
        else {
            intake1.setPower(0);
            intake2.setPower(0);
        }
    }
    public void slowmode () {
        if (gamepad1.dpad_up) {
            frontspeed = 0.25;
            sidewaysspeed = 0.5;
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
        //#1 Curveturn
        else if ((gamepad1.right_stick_x < 0 && gamepad1.left_trigger > 0) || (gamepad1.right_trigger > 0 && gamepad1.right_stick_x < 0) || gamepad1.x) {
            block.tank(0,0.5);
        }
        //#2 Curveturn
        else if ((gamepad2.right_stick_x > 0 && gamepad1.left_trigger > 0) || (gamepad1.right_trigger < 0 && gamepad1.right_stick_x > 0) || gamepad1.b) {
            block.tank(0.5,0);
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
    public void mover () {
        if (gamepad2.a) {
            mover1.setPosition(1);
            mover2.setPosition(0);

        }
        if (gamepad2.y) {
            mover1.setPosition(0);
            mover2.setPosition(1);
        }
    }

    public void linearslide () {
        //If gamepad 2 left stick pointing up
        if (gamepad2.left_stick_y < 0) {
            //Move linear slide up
            linear1.setPower(1);
            linear2.setPower(-1);
        }
        //If gamepad 2 left stick pointing up
        else if (gamepad2.left_stick_y > 0) {
            //Move linear slide down
            linear1.setPower(-0.6);
            linear2.setPower(0.6);
        }
        //If gamepad 2 left stick in center
        else if (gamepad2.left_stick_x < 0) {
            //Move linear slide up
            linear1.setPower(0.4);
            linear2.setPower(-0.4);
        }
        else if (gamepad2.left_stick_x > 0) {
            linear1.setPower(-0.2);
            linear2.setPower(0.2);
        }
        else {
            //Stop linear slide
            linear1.setPower(0);
            linear2.setPower(0);
        }
    }
    public void fold () {

    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
@Disabled
public class RIGHT_BLOCK extends LinearOpMode {
    Motor left1;
    Motor right1;
    Motor left2;
    Motor right2;
    MotorBlock block;
    Servo claw1;
    Servo claw2;
    double frontspeed = 0.4;
    double turnspeed = (0.25+0.6)/2;
    boolean isright = false;
    BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle;
    public void imuinit () {
        if (isright) imu = hardwareMap.get(BNO055IMU.class,"imu");
        else imu = hardwareMap.get(BNO055IMU.class,"imu");
        parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu.initialize(parameters);

        telemetry.addData("Gyro Mode", "calibrating...");

        telemetry.update();


        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Gyro Mode", "ready");

        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());

        telemetry.update();

    }
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    public void runOpMode() throws InterruptedException {
        //Variable Definitions
        left1 = new Motor(hardwareMap.get(DcMotor.class, "left1"));
        right1 = new Motor(hardwareMap.get(DcMotor.class, "right1"));
        left2 = new Motor(hardwareMap.get(DcMotor.class, "left2"));
        right2 = new Motor(hardwareMap.get(DcMotor.class, "right2"));
        block = new MotorBlock(left1,right1,left2,right2);
        claw1 = hardwareMap.get(Servo.class,"claw1");
        claw2 = hardwareMap.get(Servo.class,"claw2");

        imuinit();

        waitForStart();
        resetAngle();

        Motor turnclaw = new Motor(hardwareMap.get(DcMotor.class, "turnclaw"));
        turnclaw.setPower(0.5);
        sleep(900);
        turnclaw.setPower(0);
        claw1.setPosition(0);
        claw2.setPosition(0);

        sleep(750);

        block.forwardrotations(0.75,25);

        block.stop();

        sleep(500);

        block.left1.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        block.right1.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        block.right2.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        block.left2.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (!isright) {
            block.right(turnspeed);
            while (opModeIsActive() && getAngle() > 12.5) {
                telemetry.addData("angle",getAngle());
                telemetry.update();
            }
        }

        block.stop();
        sleep(750);

        claw1.setPosition(1);
        claw2.setPosition(1);

        sleep(750);

        block.backwardrotations(0.75,23);

        block.stop();
        sleep(500);

        //resetAngle();
        block.left1.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        block.right1.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        block.right2.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        block.left2.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (!isright) {
            block.left(turnspeed);
            while (opModeIsActive() && getAngle() < 70) {
                telemetry.addData("angle",getAngle());
                telemetry.update();
            }
        }

        block.stop();

        block.forwardrotations(0.75,60);

        claw1.setPosition(0);
        claw2.setPosition(0);

        block.backwardrotations(0.75,20);

        //block.leftsidewaysrotations(0.75,20);
    }
}


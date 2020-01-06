package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
public class RIGHT_FOUNDATION extends LinearOpMode {
    Motor left1;
    Motor right1;
    Motor left2;
    Motor right2;
    MotorBlock block;
    double frontspeed = 0.4;
    double turnspeed = 0.5;
    BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    Orientation lastAngles = new Orientation();
    double                  globalAngle;
    public void imuinit () {
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu.initialize(parameters);

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
        Servo mover1 = hardwareMap.get(Servo.class,"mover1");
        Servo mover2 = hardwareMap.get(Servo.class,"mover2");

        imuinit();
        resetAngle();

        waitForStart();

        block.backwardrotations(frontspeed,34);

        block.stop();

        mover1.setPosition(1);
        mover2.setPosition(0);

        sleep(300);

        /*block.frontbla(frontspeed);
        sleep(1500);*/

        block.forwardrotations(frontspeed,33.5);

        /*block.right(turnspeed);
        sleep(500);*/

        block.stop();

        sleep(1000);

        mover1.setPosition(0);
        mover2.setPosition(1);

        resetAngle();
        block.left1.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        block.right1.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        block.right2.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        block.left2.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        block.left(turnspeed);
        while (opModeIsActive() && getAngle() == 0) {
            telemetry.addData("angle",getAngle());
            telemetry.update();
        }

        while (opModeIsActive() && getAngle() < 90) {
            telemetry.addData("angle",getAngle());
            telemetry.update();
        }

        block.forwardrotations(frontspeed,25);

        block.leftsidewaysrotations(0.5,13.5,imu);

        block.tank(-turnspeed/4,-turnspeed*1);
        sleep(1600);

        block.backwardrotations(frontspeed,20);

        block.forwardrotations(frontspeed,31);
    }
}


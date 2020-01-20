package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
public class SKYSTONE_LEFT extends LinearOpMode {
    ColorSensor colorSensor;
    Motor left1;
    Motor right1;
    Motor left2;
    Motor right2;
    MotorBlock block;
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
    public void runOpMode () throws InterruptedException {
        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        left1 = new Motor(hardwareMap.get(DcMotor.class, "left1"));
        right1 = new Motor(hardwareMap.get(DcMotor.class, "right1"));
        left2 = new Motor(hardwareMap.get(DcMotor.class, "left2"));
        right2 = new Motor(hardwareMap.get(DcMotor.class, "right2"));
        block = new MotorBlock(left1, right1, left2, right2);
        int skystone = 0;
        int stone  = 0;
        telemetry.addData("1","Show the skystone (press x when done):");
        while (!(gamepad1.x || gamepad2.x)) {
            telemetry.addData("1","Show the skystone (press x when done):");
            telemetry.addData("2",colorSensor.argb());
            telemetry.update();
        }
        skystone = colorSensor.argb();
        telemetry.addData("1","Show the normal stone (press x when done):");
        sleep(500);
        while (!(gamepad1.x || gamepad2.x)) {
            telemetry.addData("1","Show the skystone (press x when done):");
            telemetry.addData("2",colorSensor.argb());
            telemetry.addData("3","skystone val is"+skystone);
            telemetry.update();
        }
        stone = colorSensor.argb();
        telemetry.addData("1","skystone is "+skystone,"stone is "+stone);
        telemetry.update();
        int median = (stone+skystone)/2;

        imuinit();
        waitForStart();

        block.forwardrotations(1,15);

        block.right(1);
        while (opModeIsActive() && getAngle() == 0) {
            telemetry.addData("angle",getAngle());
            telemetry.update();
        }

        while (opModeIsActive() && getAngle() > -90) {
            telemetry.addData("angle",getAngle());
            telemetry.update();
        }
        telemetry.addData("done","done");
        telemetry.update();
        block.forward(0.5);
        while (colorSensor.argb() > median) {

        }
        sleep(100);
        block.stop();
    }
}

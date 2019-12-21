package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class rotationstest extends LinearOpMode {
    public void runOpMode () {
        DcMotor letstestthis = hardwareMap.get(DcMotor.class,"letstestthis");
        waitForStart();
        letstestthis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        letstestthis.setPower(0.5);
        letstestthis.setTargetPosition(-1120);
        letstestthis.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (letstestthis.isBusy() && opModeIsActive()) {
            telemetry.addData("Ticks: ", letstestthis.getCurrentPosition());
            telemetry.update();

        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@Autonomous
@Disabled
public class colorsensor extends LinearOpMode {
    public void runOpMode () {
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "color");
        waitForStart();
        while (opModeIsActive()) {
            int a = colorSensor.argb();
            telemetry.addData("value", a);
            telemetry.update();
        }
    }
}

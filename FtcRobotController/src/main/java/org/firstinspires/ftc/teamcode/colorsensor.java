package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@Autonomous
public class colorsensor extends LinearOpMode {
    public void runOpMode () {
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "color");
        waitForStart();
        while (opModeIsActive()) {
            int alpha = colorSensor.alpha();
            int blue = colorSensor.blue();
            int green = colorSensor.green();
            int red = colorSensor.red();
            int rgba = colorSensor.argb();
            telemetry.addData("alpha",alpha);
            telemetry.addData("red",red);
            telemetry.addData("blue",blue);
            telemetry.addData("green",green);
            telemetry.addData("rgba",rgba);
            if (rgba < 70000000) telemetry.addData("Skystone","Skystone");
            else telemetry.addData("No Skystone","No Skystone");
            telemetry.update();
        }
    }
}

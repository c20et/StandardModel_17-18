package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name = "Color Sensor", group = "Sensor")
@Disabled
public class ColorSensorTest extends LinearOpMode {

    ColorSensor colorSensorR;
    ColorSensor colorSensorL;
    // Hardware Device Object


    @Override
    public void runOpMode() {

        colorSensorR = hardwareMap.get(ColorSensor.class, "color sensor right");
        colorSensorL = hardwareMap.get(ColorSensor.class, "color sensor left");

        // wait for the start button to be pressed.
        waitForStart();

        while (opModeIsActive()) {

            // send the info back to driver station using telemetry function.
            telemetry.addData("RGB VALS (R): ", "red (%.2f), blue (%.2f), green (%.2f)", colorSensorR.red() , colorSensorR.blue(), colorSensorR.green());
            telemetry.addData("RGB VALS (L): ", "red (%.2f), blue (%.2f), green (%.2f)", colorSensorL.red() , colorSensorL.blue(), colorSensorL.green());

            telemetry.update();
        }

    }
}

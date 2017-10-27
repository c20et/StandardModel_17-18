package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name = "Color Sensor ", group = "Sensor")
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
            telemetry.addData("Red  R: ", colorSensorR.red());
            telemetry.addData("Green R: ", colorSensorR.green());
            telemetry.addData("Blue R: ", colorSensorR.blue());

            telemetry.addData("Red  L: ", colorSensorL.red());
            telemetry.addData("Green L: ", colorSensorL.green());
            telemetry.addData("Blue L: ", colorSensorL.blue());

            telemetry.update();
        }

    }
}

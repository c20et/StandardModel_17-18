package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Color Sensor Color", group = "Sensor")
public class ColorSensorTest extends LinearOpMode {

    ColorSensor colorSensorR;
    ColorSensor colorSensorL;


    @Override
    public void runOpMode() {
        colorSensorR = hardwareMap.get(ColorSensor.class, "color sensor right");
        colorSensorL = hardwareMap.get(ColorSensor.class, "color sensor left");

        while(opModeIsActive()) {
            telemetry.addData("RGB VALS (R)", "red (%.2f), blue (%.2f), green (%.2f)", colorSensorR.red() , colorSensorR.blue(), colorSensorR.green());
        }


    }


}


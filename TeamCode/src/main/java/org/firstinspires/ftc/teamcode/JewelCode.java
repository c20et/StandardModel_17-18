package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Jewel Color", group = "Sensor")

public class JewelCode extends LinearOpMode {

    ColorSensor colorSensorR;
    ColorSensor colorSensorL;
    Servo loweringJewelServo;
    Servo turningJewelServo;

    public double loweredPosition = 0.7;
    public double leftPosition = 0.7;
    public double rightPosition = 0.;
    public direction turnDirection;
    ElapsedTime runtime;

    @Override
    public void runOpMode() {
        colorSensorR = hardwareMap.get(ColorSensor.class, "color sensor right");
        colorSensorL = hardwareMap.get(ColorSensor.class, "color sensor left");

        waitForStart();
        runtime.reset();

        boolean finishedHitting = false;
        lowerJewelMech();

        while (opModeIsActive() && !finishedHitting) {
            RedJewelLocation redLocation = getColor();
            if(redLocation != RedJewelLocation.INCONCLUSIVE) {
                chooseTurnDirection(redLocation);
                turn();
                finishedHitting = true;
            }
        }
    }

    public void chooseTurnDirection(RedJewelLocation redLocation) {
        if (redLocation == RedJewelLocation.RED_JEWEL_RIGHT) {
            if(getAlliance() == TeamAlliance.Blue) {
                turnDirection = turnDirection.Right;
            } else {
                turnDirection = turnDirection.Left;
            }
        } else if(redLocation == RedJewelLocation.RED_JEWEL_LEFT) {
            if(getAlliance() == TeamAlliance.Red) {
                turnDirection = turnDirection.Right;
            } else {
                turnDirection = turnDirection.Left;
            }
        }
    }

    public RedJewelLocation getColor() {
        if (colorSensorR.red() > colorSensorL.red() || colorSensorL.blue() > colorSensorR.blue() || colorSensorR.red() > colorSensorR.blue()) {
            return RedJewelLocation.RED_JEWEL_RIGHT;
        }
        if (colorSensorL.red() > colorSensorR.red() || colorSensorR.blue() > colorSensorR.red() || colorSensorL.red() > colorSensorR.blue()) {
            return RedJewelLocation.RED_JEWEL_LEFT;
        }
        return RedJewelLocation.INCONCLUSIVE;
    }

    public enum RedJewelLocation {
        RED_JEWEL_LEFT, RED_JEWEL_RIGHT, INCONCLUSIVE
    }

    public enum TeamAlliance {
        Red, Blue
    }

    public void lowerJewelMech() {
        loweringJewelServo.setPosition(loweredPosition);
    }

    public enum direction {
        Right, Left
    }

    public void turn() {
        if (turnDirection == direction.Right) {
            turningJewelServo.setPosition(rightPosition);
        }
        else{
            turningJewelServo.setPosition(leftPosition);
        }
    }

    public TeamAlliance getAlliance() {
        return TeamAlliance.Red;
    }
}




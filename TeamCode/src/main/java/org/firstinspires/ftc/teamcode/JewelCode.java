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

    ColorSensor colorSensorL;
    Servo loweringJewelServo;
    Servo turningJewelServo;

    public double loweredPosition = 0.2;
    public double upPosition = 0.7;
    public double leftPosition = .5;
    public double rightPosition = 1.0;
    public direction turnDirection;

    @Override
    public void runOpMode() {
        colorSensorL = hardwareMap.get(ColorSensor.class, "color sensor left");
        loweringJewelServo = hardwareMap.get(Servo.class, "lowering servo" );
        turningJewelServo = hardwareMap.get(Servo.class, "turning servo");

        waitForStart();

        boolean finishedHitting = false;
        lowerJewelMech();

        while (opModeIsActive() && !finishedHitting) {

            RedJewelLocation redLocation = getColor();
            if(redLocation != RedJewelLocation.INCONCLUSIVE) {
                chooseTurnDirection(redLocation);

                turn();
                telemetry.update();

                finishedHitting = true;
            }
        }
        loweringJewelServo.setPosition(upPosition);
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
        if (colorSensorL.blue() > 0) {

            telemetry.addLine("Right");
            telemetry.update();

            return RedJewelLocation.RED_JEWEL_RIGHT;
        }
        if (colorSensorL.red() > 0) {

            telemetry.addLine("Left");
            telemetry.update();

            return RedJewelLocation.RED_JEWEL_LEFT;
        }

        telemetry.addLine("INCONCLUSIVE");
        telemetry.update();

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

        telemetry.addLine("Servo lowered");
        telemetry.update();
    }

    public enum direction {
        Right, Left
    }

    public void turn() {

        if (turnDirection == direction.Right) {
            while (turningJewelServo.getPosition() != rightPosition) {
                telemetry.addData("Servo Position: ", turningJewelServo.getPosition());
                telemetry.addLine("Reached");

                turningJewelServo.setPosition(rightPosition);
            }

        }
        else{
            while (turningJewelServo.getPosition() != leftPosition) {
                telemetry.addData("Servo Position: ", turningJewelServo.getPosition());
                telemetry.addLine("Reached");

                turningJewelServo.setPosition(leftPosition);
            }
        }

    }

    public TeamAlliance getAlliance() {
        return TeamAlliance.Red;
    }
}




package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Servo Test", group="Linear Opmode")
public class ServoTest extends LinearOpMode {

    Servo rightTop;
    Servo leftTop;
    Servo jewelservo;
    Servo rightBottom;
    Servo leftBottom;

    //static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int CYCLE_MS = 500;     // period of each cycle

    static final double OPEN_POS_ARM = 0.80;
    static final double CLOSE_POS_ARM = 0.40;

    static final double OPEN_BOTTOM_CLAW = 0.0;
    static final double CLOSE_BOTTOM_CLAW = 0.53;

    public static double posRT = .1;
    public static double posLT = .9;
    public static double posRB = .9;
    public static double posLB = .1;


    //Starting claw positions
//    double  topServoPosition = (UP_POS_SHOULDER);
//    double  tServoPosition = (OPEN_POS_ARM);

    @Override
    public void runOpMode() {
        rightTop = hardwareMap.get(Servo.class, "right arm servo");
        leftTop = hardwareMap.get(Servo.class, "left arm servo");

        jewelservo = hardwareMap.get(Servo.class, "lowering servo");

        leftBottom = hardwareMap.get(Servo.class, "left bottom claw");
        rightBottom = hardwareMap.get(Servo.class, "right bottom claw");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        boolean rightBump = false;
        boolean leftBump = false;

        // Wait for the game to start (driver presses PLAY)

        setServos();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData("Right Top: ", rightTop.getPosition());
            telemetry.addData("Left Top: ", leftTop.getPosition());
            telemetry.addData("Right Bottom: ", rightBottom.getPosition());
            telemetry.addData("Left Bottom: ", leftBottom.getPosition());

            if (gamepad2.a) {
                posRT += .1;
            }
            if (gamepad2.b) {
                posLT -= .1;
            }
            if (gamepad2.x) {
                posRB -= .1;
            }
            if (gamepad2.y) {
                posLB += .1;
            }

            setServos();
            telemetry.update();

            sleep(CYCLE_MS);
            idle();
        }
    }

    public void setServos() {
        rightTop.setPosition(posRT);
        leftTop.setPosition(posLT);

        rightBottom.setPosition(posRB);
        leftBottom.setPosition(posLB);
    }
}
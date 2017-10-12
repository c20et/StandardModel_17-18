//same as Mecanum Test

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Mecanum Test", group="Linear Opmode")
public class MecanumTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontLeftDrive = null;
    private DcMotor FrontRightDrive = null;
    private DcMotor BackLeftDrive = null;
    private DcMotor BackRightDrive = null;
    private DcMotor LiftDrive = null;
    Servo clawservo;
    Servo jewelservo;

    //static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS_CLAW     =  1.0;     // Maximum rotational position
    static final double MIN_POS_CLAW     =  0.0;     // Minimum rotational position
    static final double MAX_POS_JEWEL     =  1.0;     // Maximum rotational position
    static final double MIN_POS_JEWEL     =  0.0;
    static final double strafepower = 0.50;

    // Define class members
    double  cservoposition = (MAX_POS_CLAW - MIN_POS_CLAW) / 2; // Start at halfway position
    double  jservoposition = (MAX_POS_JEWEL - MIN_POS_JEWEL) / 2; // Start at halfway position
    boolean rampUp = true;

    @Override
    public void runOpMode() {
        clawservo = hardwareMap.get(Servo.class, "claw servo");
        jewelservo = hardwareMap.get(Servo.class, "jewel servo");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note thl at the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller
        // pp on the phone).
        FrontLeftDrive = hardwareMap.get(DcMotor.class, "front_left");
        FrontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        BackLeftDrive = hardwareMap.get(DcMotor.class, "back_left");
        BackRightDrive = hardwareMap.get(DcMotor.class, "back_right");
        LiftDrive = hardwareMap.get(DcMotor.class, "lift");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        FrontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        BackLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        BackRightDrive.setDirection(DcMotor.Direction.REVERSE);
        FrontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        LiftDrive.setDirection(DcMotor.Direction.FORWARD);

        FrontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        boolean abuttonpressed = true;
        int cservopos = 1;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
//
            //SERVO CONTROL
            if(gamepad1.right_bumper || gamepad1.left_bumper) {
                clawservo.setPosition(0);
            }
            else {
                clawservo.setPosition(1);
            }

            if(gamepad1.a) {
                jewelservo.setPosition(1);
            } else if(gamepad1.b) {
                jewelservo.setPosition(.35);
            }
//            if(abuttonpressed) {
//                if(gamepad1.a) {
//                    jewelservo.setPosition(1);
//                    abuttonpressed = false;
//                }
//            }
//            if() {
//                jewelservo.setPosition(1);
//            }

            //DRIVING CONTROL
            double drive = -1 * gamepad1.left_stick_y; // -1 is because the robot drives backwardks when the joystick is pushed downwards
//            double strafe  =  gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            int strafedirection;
            if(Math.abs(drive) < .1 && gamepad1.right_trigger > .1) {
                strafedirection = 1;
                Strafe(strafedirection);
            }
            else if((Math.abs(drive) < .1) && gamepad1.left_trigger > .1 ) {
                strafedirection = -1;
                Strafe(strafedirection);
            }
            else if(drive > 0.1 || drive < - 0.1 || turn > 0.1 || turn < -0.1) {
                Drive(drive, turn);
            }
            else {
                FrontLeftDrive.setPower(0);
                BackLeftDrive.setPower(0);
                FrontRightDrive.setPower(0);
                BackRightDrive.setPower(0);
            }

            //LIFT MOTOR CONTROL
            if(gamepad1.x) {
                LiftDrive.setPower(0.5);
            } else if(gamepad1.y) {
                LiftDrive.setPower(-0.5);
            }else {
                LiftDrive.setPower(0);
            }


            sleep(CYCLE_MS);
            idle();
        }
    }

    public void Strafe(int strafedirection) {
        double FRpower = -strafedirection * strafepower;
        double BLpower =  -strafedirection * strafepower;
        double BRpower = strafedirection * strafepower;
        double FLpower =  strafedirection * strafepower ;

        FLpower = Range.clip(FLpower, -.75, .75) ;
        BRpower = Range.clip(BRpower, -.75, .75) ;
        BLpower = Range.clip(BLpower, -.75, .75) ;
        FRpower = Range.clip(FRpower, -.75, .75) ;

        FrontLeftDrive.setPower(FLpower);
        BackLeftDrive.setPower(BLpower);
        FrontRightDrive.setPower(FRpower);
        BackRightDrive.setPower(BRpower);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "front left (%.2f), front right (%.2f)", FLpower, FRpower);
        telemetry.addData("Motors", "back left (%.2f), back right (%.2f)", BLpower, BRpower);
        telemetry.update();


    }
    public void Drive(double drive, double turn) {
        double FRpower = drive - turn;
        double BRpower = drive - turn;
        double FLpower = drive + turn;
        double BLpower = drive + turn;

        FLpower = Range.clip(FLpower, -1.0, 1.0) ;
        BRpower = Range.clip(BRpower, -1.0, 1.0) ;
        BLpower = Range.clip(BLpower, -1.0, 1.0) ;
        FRpower = Range.clip(FRpower, -1.0, 1.0) ;

        //if a motor power is less than .1 and more than -.1, set to 0

        if(FLpower < 0.1 && FLpower > -0.1 ) {
            FLpower = 0;
        }
        if(BRpower < 0.1 && BRpower > -0.1 ) {
            BRpower = 0;
        }
        if(BLpower < 0.1 && BLpower > -0.1 ) {
            BLpower = 0;
        }
        if(FRpower < 0.1 && FRpower > -0.1 ) {
            FRpower = 0;
        }

        FrontLeftDrive.setPower(FLpower);
        BackLeftDrive.setPower(BLpower);
        FrontRightDrive.setPower(FRpower);
        BackRightDrive.setPower(BRpower);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "front left (%.2f), front right (%.2f)", FLpower, FRpower);
        telemetry.addData("Motors", "back left (%.2f), back right (%.2f)", BLpower, BRpower);
        telemetry.update();
    }

}
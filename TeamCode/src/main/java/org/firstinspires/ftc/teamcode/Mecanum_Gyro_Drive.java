package org.firstinspires.ftc.teamcode;



import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@TeleOp(name="Mecanum_Gyro_Drive", group="Linear Opmode")
public class Mecanum_Gyro_Drive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    public double frontLeftPower;
    public double frontRightPower;
    public double backLeftPower;
    public double backRightPower;
    public double straftingSpead = .75;
    public int straftingDirection = 1;
    // The IMU sensor object
    BNO055IMU imu;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightMotor");
//        internalGyro = hardwareMap.get(InternalGyroSensor.class, "");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        // CHECK ONCE BUILT
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Wait until we're told to go

            // Setup a variable for each drive wheel to save power level for telemetry
            straftingSpead = .75;


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

//            double straftingSpead = -gamepad1.left_stick_x;

            frontLeftPower = Range.clip(drive + turn, -1.0, 1.0);
            backLeftPower = frontLeftPower;
            frontRightPower = Range.clip(drive - turn, -1.0, 1.0);
            backRightPower = frontRightPower;

            if (gamepad1.right_bumper) {
                straftingDirection = 1;
                straftingSpead(straftingDirection);
            }
            else if (gamepad1.left_bumper) {
                straftingDirection = -1;
                straftingSpead(straftingDirection);
            }

            // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeftPower);
            backLeftDrive.setPower(backLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backRightDrive.setPower(backRightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "front left (%.2f), front right (%.2f), back left (%.2f), back right (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.update();
        }
    }

    public void straftingSpead(int sDirection) {
        frontLeftPower = sDirection*straftingSpead;
        backLeftPower = -sDirection*straftingSpead;
        frontRightPower = -sDirection*straftingSpead;
        backRightPower = sDirection*straftingSpead;
    }
}

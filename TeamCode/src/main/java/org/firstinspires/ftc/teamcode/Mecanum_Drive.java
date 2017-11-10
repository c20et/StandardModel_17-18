package org.firstinspires.ftc.teamcode;



import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@TeleOp(name="Mecanum_Drive", group="Linear Opmode")

public class Mecanum_Drive extends LinearOpMode {

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
    public double currentAngle;
    // The gyro sensor
    public BNO055IMU imu;

    // State used for updating telemetry
    public Orientation angles;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right");

//        internalGyro = hardwareMap.get(InternalGyroSensor.class, "");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        // CHECK ONCE BUILT
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Wait until we're told to go
            // Setup a variable for each drive wheel to save power level for telemetry

            //OPTION FOR DRIVER CONTROL -- ONLY IF THEY WANT
//            double straftingSpead = -gamepad1.left_stick_x;

            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            formatAngle(angles.angleUnit, angles.firstAngle);

            final double initialAngle = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));

            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            if(drive > .1) {
                while(drive > .1) {
                    forwardDrive(drive, initialAngle);
                }
            }
            else if (turn > .1) {
                while(turn > .1 ) {
                    turningDrive();
                }
            }
            while (gamepad1.right_bumper) {
                straftingDirection = 1;
                straftingSpead();
            }
            while (gamepad1.left_bumper) {
                straftingDirection = -1;
                straftingSpead();
            }

            // Send calculated power to wheels
            move();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "front left (%.2f), front right (%.2f), back left (%.2f), back right (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.addData("Gamepad", "left_stick_y (%.2f), right_stick_x (%.2f)", -gamepad1.left_stick_y, gamepad1.right_stick_x);
//            telemetry.addData("Angular Heading", new Func<String>() {
//                @Override public String value() {
//                    return formatAngle(angles.angleUnit, angles.firstAngle);
//                }
//            });
            telemetry.update();
        }
    }

    //worry
    public void move() {
        frontLeftDrive.setPower(frontLeftPower);
        backLeftDrive.setPower(backLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backRightDrive.setPower(backRightPower);
        currentAngle = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
    }

    //dont worry
    public void straftingSpead() {
        frontLeftPower = straftingDirection*straftingSpead;
        backLeftPower = -straftingDirection*straftingSpead;
        frontRightPower = -straftingDirection*straftingSpead;
        backRightPower = straftingDirection*straftingSpead;
    }

    public void forwardDrive(double drive, double initialAngle) {
/*        frontLeftPower = Range.clip(drive + turn, -1.0, 1.0);
        backLeftPower = frontLeftPower;
        frontRightPower = Range.clip(drive - turn, -1.0, 1.0);
        backRightPower = frontRightPower;
*/
        //Drives forward
        frontLeftPower = Range.clip(drive, -1.0, 1.0);
        backLeftPower = frontLeftPower;
        frontRightPower = Range.clip(drive, -1.0, 1.0);
        backRightPower = frontRightPower;
//        move();
//        if(Math.abs(currentAngle-initialAngle) > 2) {
//            moveToHeading(initialAngle);
//        }
    }

    public void turningDrive () {
        double turn = gamepad1.right_stick_x;
        frontLeftPower = Range.clip(turn, -1.0, 1.0);
        backLeftPower = frontLeftPower;
        frontRightPower = Range.clip(-turn, -1.0, 1.0);
        backRightPower = frontRightPower;
    }

    public void turnToHeading (double heading, double initialAngle) {
        double powerMultipler = 1;
        currentAngle = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        double angleDiffrence = Math.abs(currentAngle-initialAngle);
        while(angleDiffrence > 1 ) {
            frontLeftPower = .5 * powerMultipler;
            backLeftPower = frontLeftPower;
            frontRightPower = -.5 * powerMultipler;
            backRightPower = frontRightPower;
            move();
            angleDiffrence = Math.abs(currentAngle-initialAngle);
        }
    }

    public void moveToHeading(double initialAngle) {
        boolean turningToRight;
        if(currentAngle-initialAngle > 0 ) {
            turningToRight = true;
        }
        else {
            turningToRight = false;
        }
        while(Math.abs(currentAngle-initialAngle) > 1 ) {
            if(turningToRight) {
                frontLeftPower = .5;
                backLeftPower = .5;
                frontRightPower = .3;
                backRightPower = .3;
            }
            else {
                frontLeftPower = .3;
                backLeftPower = .3;
                frontRightPower = .5;
                backRightPower = .5;
            }
            move();
        }
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}

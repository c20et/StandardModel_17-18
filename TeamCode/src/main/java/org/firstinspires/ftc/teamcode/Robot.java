package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Locale;

public class Robot {
    // Motors
    public static DcMotor frontLeftDrive;
    public static DcMotor frontRightDrive = null;
    private static DcMotor backLeftDrive = null;
    private static DcMotor backRightDrive = null;
    private static DcMotor liftDrive = null;

    public static DcMotor nom;

    public static DcMotor conveyor;

    // Servos
    public static Servo clawServo;
    public static Servo jewelServo;

    // Sensors
    public static ColorSensor frontJewelColor = null;
    public static ColorSensor currentBlockColor = null;
    public static BNO055IMU imu; //gyro
    public static Orientation angles;
    public static Acceleration gravity;

    //Vuforia
    public static final String TAG = "Vuforia VuMark Sample";
    public OpenGLMatrix lastLocation = null;
    public VuforiaLocalizer vuforia;
    public static VoltageSensor voltageSensor;

    // Telemetry
    public static Telemetry telemetry;

    // The original opmode.
    public static LinearOpMode opMode;

    public static void init(LinearOpMode om) {
        opMode = om;
        HardwareMap hardwareMap = opMode.hardwareMap;

        // Motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right");
        liftDrive = hardwareMap.get(DcMotor.class, "lift");

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        liftDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servos
        clawServo = hardwareMap.get(Servo.class, "claw servo");
        jewelServo = hardwareMap.get(Servo.class, "jewel servo");

        // Sensors:
        // Color
        currentBlockColor = hardwareMap.colorSensor.get("block color");
        currentBlockColor.setI2cAddress(I2cAddr.create8bit(0x4C));
        currentBlockColor.enableLed(true);

        frontJewelColor = hardwareMap.colorSensor.get("jewel color");
        frontJewelColor.enableLed(false);

        // IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;

        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        composeTelemetry();

        // Vuforia
        // TODO: ADD THIS CODE IN, IT IS IN  VMARKIMAGEIDENTIFICATION

        // Voltage
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public static void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    public static String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public static String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public static void Strafe(int strafedirection, double strafepower) {
        double FRpower = -strafedirection * strafepower;
        double BLpower =  -strafedirection * strafepower;
        double BRpower = strafedirection * strafepower;
        double FLpower =  strafedirection * strafepower ;

        //in case accidentally a different power is used.
        FLpower = Range.clip(FLpower, -1, 1) ;
        BRpower = Range.clip(BRpower, -1, 1) ;
        BLpower = Range.clip(BLpower, -1, 1) ;
        FRpower = Range.clip(FRpower, -1, 1) ;

        frontLeftDrive.setPower(FLpower);
        backLeftDrive.setPower(BLpower);
        frontRightDrive.setPower(FRpower);
        backRightDrive.setPower(BRpower);


    }
}

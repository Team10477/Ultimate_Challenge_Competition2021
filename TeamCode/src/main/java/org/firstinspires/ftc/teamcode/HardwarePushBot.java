package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * These are the hardware configuration of our 2021 robot.
 */
public class HardwarePushBot {

    /**
     * Four mecanum wheels.
     **/
    public DcMotor leftBackWheel = null;
    public DcMotor rightBackWheel = null;
    public DcMotor leftFrontWheel = null;
    public DcMotor rightFrontWheel = null;

/*
    Intake, Shooting, and Wobble Goal DC Motors
 */
    public DcMotor ringIntake = null;
    public DcMotor shootingWheel = null;
    public DcMotor wobbleGoalArm = null;

    /**
     * Two color Sensors
     */
    public RevColorSensorV3 leftColorSensor = null;
    public RevColorSensorV3 rightColorSensor = null;
    public RevColorSensorV3 frontColorSensor = null;

    /**
     * Touch Sensors
     **/
    public DigitalChannel touchSensorRight = null;
    public DigitalChannel touchSensorWaFront = null;
    public DigitalChannel touchSensorWaBack = null;

    /**
     * Servos
     **/
    public Servo wobbleGoalFinger = null;
    public Servo shootingTrigger = null;
    public Servo wobbleGoalFinger2 = null;

   /* double leftFrontPower = 0;
    double rightFrontPower = 0;
    double leftRearPower = 0;
    double rightRearPower = 0;*/

    /*
    IMU Sensor Related
     */
    private ElapsedTime elapsedTime = new ElapsedTime();
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle;
    double heading;
    double integralError =0;
    double error =0;
    double deltaTurn = 0;
    double GAIN_PROP = 0.015;
    double GAIN_INT = 0.015;

    public void init(HardwareMap hwMap) {
        mapWheels(hwMap);
        mapWobbleArm(hwMap);
        mapTouchSensor(hwMap);
        mapColorSensor(hwMap);
        mapRingIntake(hwMap);
        mapWobbleArm(hwMap);
        mapShootingWheel(hwMap);
    }

    public void mapWheels(HardwareMap hwMap) {
        leftBackWheel = hwMap.get(DcMotor.class, "left_rear");
        rightBackWheel = hwMap.get(DcMotor.class, "right_rear");
        leftFrontWheel = hwMap.get(DcMotor.class, "left_front");
        rightFrontWheel = hwMap.get(DcMotor.class, "right_front");
    }

    public void mapRingIntake(HardwareMap hwMap) {
        ringIntake = hwMap.get(DcMotor.class, "ring_intake");
    }
    public void mapWobbleArm(HardwareMap hwMap) {
        wobbleGoalArm = hwMap.get(DcMotor.class, "wobble_arm");
        wobbleGoalFinger = hwMap.get(Servo.class, "wobble_finger");
        wobbleGoalFinger2 = hwMap.get(Servo.class, "wobble_finger2");
    }

    public void mapShootingWheel(HardwareMap hwMap){
        shootingWheel = hwMap.get(DcMotor.class, "shooting_wheel");
        shootingTrigger = hwMap.get(Servo.class, "shooting_trigger");
    }

    public void mapTouchSensor(HardwareMap hwMap) {
         touchSensorRight = hwMap.get(DigitalChannel.class, "touch_sensor_right");
        touchSensorWaFront = hwMap.get(DigitalChannel.class, "touch_sensor_wa_front");
        touchSensorWaBack = hwMap.get(DigitalChannel.class, "touch_sensor_wa_back");
        touchSensorRight.setMode(DigitalChannel.Mode.INPUT);
        touchSensorWaFront.setMode(DigitalChannel.Mode.INPUT);
        touchSensorWaFront.setMode(DigitalChannel.Mode.INPUT);
    }

    public void mapColorSensor(HardwareMap hwMap) {
        leftColorSensor = hwMap.get(RevColorSensorV3.class, "color_sensor_left");  // Check servo config. in RC
        rightColorSensor = hwMap.get(RevColorSensorV3.class, "color_sensor_right");  // Check servo config. in RC
        frontColorSensor = hwMap.get(RevColorSensorV3.class,"color_sensor_front");

    }

    public void mecanumDrive(double drive, double strafe, double turn) {
        double leftFrontPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
        double rightFrontPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
        double leftRearPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
        double rightRearPower = Range.clip(drive - turn - strafe, -1.0, 1.0);
        setWheelPower(leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);
    }

    public void setWheelPower(double leftFrontPower, double rightFrontPower, double leftRearPower, double rightRearPower) {
        leftFrontWheel.setPower(leftFrontPower);
        rightFrontWheel.setPower(rightFrontPower);
        leftBackWheel.setPower(leftRearPower);
        rightBackWheel.setPower(rightRearPower);
    }

    public void setWheelDirection() {
        leftFrontWheel.setDirection(DcMotor.Direction.FORWARD);
        leftBackWheel.setDirection(DcMotor.Direction.REVERSE);
        rightFrontWheel.setDirection(DcMotor.Direction.REVERSE);
        rightBackWheel.setDirection(DcMotor.Direction.FORWARD);
    }



    public void enableColorSensor(RevColorSensorV3 colSensor, HardwareMap hwMap) {
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        colSensor.enableLed(true);
    }


    public void initializeImu(HardwareMap hardwareMap) {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        resetAngle();
    }
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
}

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.HardwarePushBot;

import java.util.List;

@Autonomous(name = "Identify Rings_2WG - Eashan_Nirvan")
public class RightRedTargetZones_OD_FM_2WG extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_QUAD_ELEMENT = "Quad";
    private static final String LABEL_SINGLE_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "AZvVY5H/////AAABmVIJC2J+Bk8dmED6q+xzKkUsIvfAZMxMAHooL0K1pItNOLCvp0Mq1vBnpcHiPjwlLYt5OOVpnoyE72D5Ku8iR1ahgCo9Le6ymYWRR/No45AHVGMbfpnYxmMchxYfGn/otWPMzvohNI2lyeJwmumFgHIO9nAJ7YH86HptFVWynm16S2ENaAUfBOeKxqPABdc7i8PCe7r/PMeY2gVSAp9/53S3Gi9PGvv4f2wCtlSQChSAsbdglIuy91gLnHHzR0w1itEjtRvSnronkqr/2P9xNfWcp8tN7duysWh6eQlUhAWerhD05mvmBorePfpmCfS2sWn4FTMdzR3vZZxypY1NgOx7OvYSEhPaapYcX8ydsBOb";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    //Target zone variables to keep hold of the number of rings: 1 (Target A),2 (Target B), 3 (Target C.
    private int targetZone = 0;

    private ElapsedTime elapsedTime = new ElapsedTime();
    private ElapsedTime loopTime = new ElapsedTime();

    HardwarePushBot hardwarePushBot = new HardwarePushBot();

    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 8;
    int redColorFound = 0;
    int whitecolorfound = 0;
    float hue = 0, saturation = 0, value = 0;
    boolean isStop = false;
    BNO055IMU imu;
    double heading;
    double integralError = 0;
    double error = 0;
    double deltaTurn = 0;
    double GAIN_PROP = 0.015;
    double GAIN_INT = 0.015;
    double LIGHT_INTENSITY_WHITE = 30;

    @Override
    public void runOpMode() throws InterruptedException {
        initVuforia();
        initTfod();
        activateTfod();
        hardwarePushBot.initializeImu(hardwareMap);
        hardwarePushBot.mapWheels(hardwareMap);
        hardwarePushBot.mapColorSensor(hardwareMap);
        hardwarePushBot.mapRingIntake(hardwareMap);
        hardwarePushBot.mapWobbleArm(hardwareMap);
        hardwarePushBot.mapShootingWheel(hardwareMap);

        hardwarePushBot.leftColorSensor.enableLed(true);
        hardwarePushBot.rightColorSensor.enableLed(true);

        //hardwarePushBot.setWheelDirection();
        // Note changes for Strafer Chassis below
        hardwarePushBot.leftFrontWheel.setDirection(DcMotor.Direction.FORWARD);
        hardwarePushBot.leftBackWheel.setDirection(DcMotor.Direction.REVERSE);
        hardwarePushBot.rightFrontWheel.setDirection(DcMotor.Direction.REVERSE);
        hardwarePushBot.rightBackWheel.setDirection(DcMotor.Direction.FORWARD);

        hardwarePushBot.wobbleGoalFinger.setPosition(1.0);
        hardwarePushBot.wobbleGoalFinger2.setPosition(1.0);
        waitForStart();
        int counter = 1;
        while (opModeIsActive() && counter == 1) {
            identifyRings();
            goToTargetZones();
            counter++;
        }
        deactivateTfod();
    }

    public void goToTargetZones() {
        switch (targetZone) {
            case 1:
                //Go to the right
                drivewWithFeedback_FBM(0, 0.75, 0.80);

                //Move forward fast
                drivewWithFeedback_FBM(-0.75, 0, 1.25);

                //Go to Red
                drivewWithFeedback_FBM_Colors(-0.5, 0.0, 4.0, "RED");

                //Move back a little
                drivewWithFeedback_FBM(0.4, 0, 0.2);

                //Wobble Goal arm go down
                moveWobbleGoalArm(1.0, 0.85);

                //Drop wobble goal into target zone
                hardwarePushBot.wobbleGoalFinger.setPosition(0.0);
                hardwarePushBot.wobbleGoalFinger2.setPosition(0.0);

                //Move back a little
                drivewWithFeedback_FBM(0.4, 0, 0.5);

                //Close claw
                hardwarePushBot.wobbleGoalFinger.setPosition(1.0);
                hardwarePushBot.wobbleGoalFinger2.setPosition(1.0);

                //Wobble Goal arm go up
                moveWobbleGoalArm(-1.0, 0.85);

                //Turn on shooting wheel
                startShootingWheelByVoltage();

                sleep(500);

                //Go to the left
                drivewWithFeedback_FBM(0, -0.75, 0.75);

                //Move forward a little
                drivewWithFeedback_FBM(-0.5, 0, 0.15);

                //Shoot rings
                shootRingsIntoHighGoal();

                //Go to the left
                drivewWithFeedback_FBM(0, -0.75, 1.75);

                hardwarePushBot.shootingWheel.setPower(0);

                //Move back fast
                drivewWithFeedback_FBM(0.75, 0, 1.357);

                //Open claw
                hardwarePushBot.wobbleGoalFinger.setPosition(0.0);
                hardwarePushBot.wobbleGoalFinger2.setPosition(0.0);

                //Wobble Goal arm go down
                moveWobbleGoalArm(1.0, 1.09);

                //Turn
                mecanumDriveWithTimeout(0,0,-0.5, 1.17);

                //Move Forward
                mecanumDriveWithTimeout(-0.5,0,0,0.6);

                //Close claw
                hardwarePushBot.wobbleGoalFinger.setPosition(1.0);

                //Close second claw
                hardwarePushBot.wobbleGoalFinger2.setPosition(1.0);

                sleep(500);

                //Wobble Goal arm go up
                moveWobbleGoalArm(-1.0, 0.9);

                //Turn to the left
                mecanumDriveWithTimeout(0, 0,0.5, 0.65);

                //Move forward without feed back movement
                mecanumDriveWithTimeout(-0.85,0,0,1.545);

                //Wobble Goal arm go down
                moveWobbleGoalArm(1.0, 0.85);

                //Open claw
                hardwarePushBot.wobbleGoalFinger.setPosition(0.0);

                hardwarePushBot.wobbleGoalFinger2.setPosition(0.0);

                sleep(500);

                //Move Back
                mecanumDriveWithTimeout(0.5,0,0,0.8);

                //Turn to the left
                mecanumDriveWithTimeout(0, 0,0.5, 0.55);

                //Move forward fast
                drivewWithFeedback_FBM(-0.75, 0, 0.6);
                break;
            case 2:
                //Go to the right
                drivewWithFeedback_FBM(0, 0.75, 0.75);

                //Move forward fast
                drivewWithFeedback_FBM(-0.75, 0, 1.25);

                //Go to Red
                drivewWithFeedback_FBM_Colors(-0.5, 0.0, 4.0, "RED");

                //Go to Red
                drivewWithFeedback_FBM_Colors(-0.5, 0.0, 4.0, "RED");

                //Move Forward
                drivewWithFeedback_FBM(-0.75, 0, 0.3);

                //Go to Red
                drivewWithFeedback_FBM_Colors(0, -0.5, 4.0, "RED");


                //Move back
                drivewWithFeedback_FBM(0.75, 0, 0.5);

                //Wobble Goal arm go down
                moveWobbleGoalArm(1.0, 0.85);

                //Drop wobble goal into target zone
                hardwarePushBot.wobbleGoalFinger.setPosition(0.0);

                //Move back a little
                drivewWithFeedback_FBM(0.4, 0, 0.5);

                //Close claw
                hardwarePushBot.wobbleGoalFinger.setPosition(1.0);

                //Wobble Goal arm go up
                moveWobbleGoalArm(-1.0, 0.85);

                //Turn on shooting wheel for high goal
                hardwarePushBot.shootingWheel.setPower(1);

                //Move to white
                drivewWithFeedback_FBM_Colors(0.5, 0.0, 2.5, "WHITE");

                //Move back a little
                drivewWithFeedback_FBM(0.6, 0, 0.45);

                //Go to the right
                drivewWithFeedback_FBM(0, 0.4, 0.5);

                //Shoot rings
                shootRingsIntoHighGoal();

                //Move forward a little
                drivewWithFeedback_FBM(-0.5, 0, 0.4);

                //Go to white
                drivewWithFeedback_FBM_Colors(0.0, 0.5, 4.0, "WHITE");
                break;
            case 3:
                //Go to the right
                drivewWithFeedback_FBM(0, 0.75, 0.75);

                //Move forward fast
                drivewWithFeedback_FBM(-0.75, 0, 1.25);

                //Go to Red
                drivewWithFeedback_FBM_Colors(-0.5, 0.0, 4.0, "RED");
                drivewWithFeedback_FBM_Colors(-0.5, 0.0, 4.0, "RED");

                //Move forward
                drivewWithFeedback_FBM(-0.5, 0, 0.8);

                //Move to red again
                //drivewWithFeedback_FBM_Colors(0.5, 0.0, 4.0, "RED");

                //Move back a little
                //drivewWithFeedback_FBM(0.4, 0, 0.5);

                //Wobble Goal arm go down
                moveWobbleGoalArm(1.0, 0.87);

                //Drop wobble goal into target zone
                hardwarePushBot.wobbleGoalFinger.setPosition(0.0);

                //Move back
                drivewWithFeedback_FBM(0.5, 0, 0.3  );

                //Close claw
                hardwarePushBot.wobbleGoalFinger.setPosition(1.0);

                //Wobble Goal arm go up
                moveWobbleGoalArm(-1.0, 0.87);

                //Move back
                drivewWithFeedback_FBM(0.75, 0, 0.5);

                //Move to red
                drivewWithFeedback_FBM_Colors(0.5, 0.0, 4.0, "RED");

                //Turn on the shooting mechanism
                hardwarePushBot.shootingWheel.setPower(1);

                //Move to white
                drivewWithFeedback_FBM_Colors(0.5, 0.0, 2.5, "WHITE");

                //Move back a little
                drivewWithFeedback_FBM(0.4, 0, 1.0);

                //Go to the left
                drivewWithFeedback_FBM(0, -0.75, 0.68);

                //Move back a little
                drivewWithFeedback_FBM(0.4, 0, 0.15);

                //Shoot all 3 rings into the high goal
                shootRingsIntoHighGoal();

                //Move forward a little
                drivewWithFeedback_FBM(-0.75, 0, 0.4);
                break;
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_QUAD_ELEMENT, LABEL_SINGLE_ELEMENT);
    }

    public void activateTfod() {
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0 / 9.0);
        }
    }

    public void identifyRings() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() == 0) {
                    // empty list.  no objects recognized.
                    telemetry.addData("TFOD", "No items detected.");
                    telemetry.addData("Target Zone", "A");
                    targetZone = 1;
                } else {
                    // list is not empty.
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());

                        // check label to see which target zone to go after.
                        if (recognition.getLabel().equals("Single")) {
                            telemetry.addData("Target Zone", "B");
                            targetZone = 2;
                        } else if (recognition.getLabel().equals("Quad")) {
                            telemetry.addData("Target Zone", "C");
                            targetZone = 3;
                        } else {
                            telemetry.addData("Target Zone", "UNKNOWN");
                        }
                    }
                }

                telemetry.update();
            }
        }
    }

    public void deactivateTfod() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public void drivewWithFeedback_FBM(double drive_power, double strafe_power, double timeOut) {
        hardwarePushBot.mecanumDrive(drive_power, strafe_power, 0.0); // pass the parameters to a mecanumDrive method

        elapsedTime.reset();
        integralError = 0;
        while (opModeIsActive() && elapsedTime.seconds() < timeOut) {
            heading = hardwarePushBot.getAngle();
            error = heading - 0; // desrired - current heading is the error
            integralError = integralError + error * 0.025;

            hardwarePushBot.mecanumDrive(drive_power, strafe_power, -(error * GAIN_PROP + integralError * GAIN_INT)); // the multiplication of 0.015 is a gain to make the turn power small (not close to 1, which is maximum)
        }
        hardwarePushBot.mecanumDrive(0, 0.0, 0.0);
    }

    public void drivewWithFeedback_FBM_Colors(double drive_power, double strafe_power, double timeOut, String colorString) {
        hardwarePushBot.mecanumDrive(drive_power, strafe_power, 0.0); // pass the parameters to a mecanumDrive method
        elapsedTime.reset();
        integralError = 0;
        while (opModeIsActive() && elapsedTime.seconds() < timeOut && !isColorFound(colorString)) {

            heading = hardwarePushBot.getAngle();
            error = heading - 0; // desrired - current heading is the error
            integralError = integralError + error * 0.025;

            hardwarePushBot.mecanumDrive(drive_power, strafe_power, -(error * GAIN_PROP + integralError * GAIN_INT)); // the multiplication of 0.015 is a gain to make the turn power small (not close to 1, which is maximum)
        }
        hardwarePushBot.mecanumDrive(0, 0.0, 0.0);
    }

    public boolean isColorFound(String colorString) {
        switch (colorString) {
            case "RED": {
                return isRedColorFound();
            }
            case "WHITE": {
                return isWhitefound();
            }
            default:
                return false;
            //Add a case for blue
        }

    }

    public boolean isRedColorFound() {
        boolean found = false;

        Color.RGBToHSV((int) (hardwarePushBot.rightColorSensor.red() * SCALE_FACTOR),
                (int) (hardwarePushBot.rightColorSensor.green() * SCALE_FACTOR),
                (int) (hardwarePushBot.rightColorSensor.blue() * SCALE_FACTOR),
                hsvValues);
        // CheckForRed is a hsv value check

        hue = hsvValues[0];
        saturation = hsvValues[1];
        value = hsvValues[2];

        telemetry.addData("Color Red", String.valueOf(hue), String.valueOf(saturation), String.valueOf(value));
        telemetry.update();

        if ((hue < 60 || hue > 320)) {
            found = true;
            sleep(50);
        }

        return found;
    }

    public boolean isWhitefound() {
        boolean found = false;
        double lightIntensity;
        //     lightIntensity = hardwarePushBot.rightColorSensor.alpha(); // total light luminosity


        Color.RGBToHSV((int) (hardwarePushBot.rightColorSensor.red() * SCALE_FACTOR),
                (int) (hardwarePushBot.rightColorSensor.green() * SCALE_FACTOR),
                (int) (hardwarePushBot.rightColorSensor.blue() * SCALE_FACTOR),
                hsvValues);
        // CheckForRed is a hsv value check

        hue = hsvValues[0];
        saturation = hsvValues[1];
        value = hsvValues[2];

        telemetry.addData("Color White", String.valueOf(hue), String.valueOf(saturation), String.valueOf(value));
        telemetry.addData("Color White value", value);
        telemetry.addData("Color White saturation", saturation);
        telemetry.update();

        if (value >= LIGHT_INTENSITY_WHITE) {
            found = true;
            sleep(20);
        }

        return found;
    }

    public void moveWobbleGoalArm(double power, double time) {
        elapsedTime.reset();
        while (opModeIsActive() && elapsedTime.seconds() < time) {
            hardwarePushBot.wobbleGoalArm.setPower(power);
        }
        hardwarePushBot.wobbleGoalArm.setPower(0);
    }

    public void shootRingsIntoHighGoal() {
        for (int i = 1; i <= 3; i++) {
            hardwarePushBot.shootingTrigger.setPosition(1.0);
            sleep(750);

            hardwarePushBot.shootingTrigger.setPosition(0.0);
            sleep(750);
        }
    }

    public void mecanumDriveWithTimeout(double drive, double strafe, double turn, double timeout) {
        elapsedTime.reset();
        integralError = 0;
        while (elapsedTime.seconds() < timeout) {
            double leftFrontPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
            double rightFrontPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
            double leftRearPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
            double rightRearPower = Range.clip(drive - turn - strafe, -1.0, 1.0);
            hardwarePushBot.setWheelPower(leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);

        }
        hardwarePushBot.mecanumDrive(0, 0.0, 0.0);

    }

    public void startShootingWheelByVoltage() {
        double battVoltage= getVoltage();
        hardwarePushBot.shootingWheel.setPower(0.75-(battVoltage-12)*(0.75-0.62)/(1.95));

    }

    public double getVoltage() {
        // Computes the current battery voltage

        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();


            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        telemetry.addData("current Battery voltage is ", result);
        telemetry.update();
        return result;
    }
}
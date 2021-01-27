package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="Encoder-Red Left Target Zones")
public class LeftRedTargetZones_Encoder extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_QUAD_ELEMENT ="Quad";
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
    double integralError =0;
    double error =0;
    double deltaTurn = 0;
    double GAIN_PROP = 0.015;
    double GAIN_INT = 0.015;
    double LIGHT_INTENSITY_WHITE = 300;
    static int DRIVE_FORWARD = 1;
    static int DRIVE_BACK = 0;
    static int STRAFE_RIGHT = 2;
    static int STRAFE_LEFT = 3;
    static int WOBBLE_ARM_DOWN = 4;
    static int WOBBLE_ARM_UP = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardwareMap();
        waitForStart();
        int counter = 1;
        while (opModeIsActive() && counter == 1) {
            identifyRings();
            goToTargetZones();
            counter++;
        }
        deactivateTfod();
    }

    public void initHardwareMap() {
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
        hardwarePushBot.wobbleGoalFinger.setPosition(1);
        hardwarePushBot.shootingTrigger.setPosition(0);
        initVuforia();
        initTfod();
        activateTfod();

        telemetry.addData("Initialization complete", "Done");
        telemetry.update();

    }

    public void stopResetEncoder() {
        hardwarePushBot.leftFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardwarePushBot.leftBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardwarePushBot.rightFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardwarePushBot.rightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardwarePushBot.wobbleGoalArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void runWithoutEncoder() {
        hardwarePushBot.leftFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardwarePushBot.leftBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardwarePushBot.rightFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardwarePushBot.rightBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void setTargetPositionOld(int position) {
        hardwarePushBot.leftFrontWheel.setTargetPosition(hardwarePushBot.leftFrontWheel.getCurrentPosition() + position);
        hardwarePushBot.leftBackWheel.setTargetPosition(hardwarePushBot.leftBackWheel.getCurrentPosition() + position);
        hardwarePushBot.rightFrontWheel.setTargetPosition(hardwarePushBot.rightFrontWheel.getCurrentPosition() + position);
        hardwarePushBot.rightBackWheel.setTargetPosition(hardwarePushBot.rightBackWheel.getCurrentPosition() + position);

    }

    public void setTargetPosition(int action, int position) {
        if (action == 0) { // Drive backward
            hardwarePushBot.leftFrontWheel.setTargetPosition(hardwarePushBot.leftFrontWheel.getCurrentPosition() + position);
            hardwarePushBot.leftBackWheel.setTargetPosition(hardwarePushBot.leftBackWheel.getCurrentPosition() + position);
            hardwarePushBot.rightFrontWheel.setTargetPosition(hardwarePushBot.rightFrontWheel.getCurrentPosition() + position);
            hardwarePushBot.rightBackWheel.setTargetPosition(hardwarePushBot.rightBackWheel.getCurrentPosition() + position);
        } else if (action == 1) { // Drive forward
            hardwarePushBot.leftFrontWheel.setTargetPosition(hardwarePushBot.leftFrontWheel.getCurrentPosition() - position);
            hardwarePushBot.leftBackWheel.setTargetPosition(hardwarePushBot.leftBackWheel.getCurrentPosition() - position);
            hardwarePushBot.rightFrontWheel.setTargetPosition(hardwarePushBot.rightFrontWheel.getCurrentPosition() - position);
            hardwarePushBot.rightBackWheel.setTargetPosition(hardwarePushBot.rightBackWheel.getCurrentPosition() - position);
        } else if (action == 2) { // Strafe right
            hardwarePushBot.leftFrontWheel.setTargetPosition(hardwarePushBot.leftFrontWheel.getCurrentPosition() - position);
            hardwarePushBot.leftBackWheel.setTargetPosition(hardwarePushBot.leftBackWheel.getCurrentPosition() + position);
            hardwarePushBot.rightFrontWheel.setTargetPosition(hardwarePushBot.rightFrontWheel.getCurrentPosition() + position);
            hardwarePushBot.rightBackWheel.setTargetPosition(hardwarePushBot.rightBackWheel.getCurrentPosition() - position);
        }  else if (action == 3) { // Strafe left
            hardwarePushBot.leftFrontWheel.setTargetPosition(hardwarePushBot.leftFrontWheel.getCurrentPosition() + position);
            hardwarePushBot.leftBackWheel.setTargetPosition(hardwarePushBot.leftBackWheel.getCurrentPosition() - position);
            hardwarePushBot.rightFrontWheel.setTargetPosition(hardwarePushBot.rightFrontWheel.getCurrentPosition() - position);
            hardwarePushBot.rightBackWheel.setTargetPosition(hardwarePushBot.rightBackWheel.getCurrentPosition() + position);
        }  else if (action == 4) { // Wobble arm down
            hardwarePushBot.wobbleGoalArm.setTargetPosition(hardwarePushBot.wobbleGoalArm.getCurrentPosition()+position);
        } else if (action == 5) { // Wobble arm up
            hardwarePushBot.wobbleGoalArm.setTargetPosition(hardwarePushBot.wobbleGoalArm.getCurrentPosition()-position);
        }

    }

    public void runToPositionMode() {
        hardwarePushBot.leftFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardwarePushBot.leftBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardwarePushBot.rightFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardwarePushBot.rightBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void runToPosition(double power) {
        runToPositionMode();

        hardwarePushBot.setWheelPower(power, power, power, power);

        while (opModeIsActive() && hardwarePushBot.leftFrontWheel.isBusy() ||
                hardwarePushBot.rightFrontWheel.isBusy() && hardwarePushBot.leftBackWheel.isBusy() &&
                hardwarePushBot.rightBackWheel.isBusy()) {
            telemetry.addData("Current position right front", hardwarePushBot.rightFrontWheel.getCurrentPosition());
            telemetry.update();
        }

        hardwarePushBot.setWheelPower(0.0, 0.0, 0.0, 0.0);

    }

    public void runToPositionForWobbleArm(double power) {
        hardwarePushBot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hardwarePushBot.wobbleGoalArm.setPower(power);

        while (opModeIsActive() && hardwarePushBot.wobbleGoalArm.isBusy() ) {
            telemetry.addData("Current position wobble arm", hardwarePushBot.wobbleGoalArm.getCurrentPosition());
            telemetry.update();
        }

        hardwarePushBot.wobbleGoalArm.setPower(0);

    }

    public void goToTargetZones() {
        switch (targetZone) {
            case 1:
                //1. Strafe left

                stopResetEncoder();

                setTargetPosition(STRAFE_LEFT,1000);

                runToPosition(1);

                //2. Go forward.

                setTargetPosition(DRIVE_FORWARD,3550);

                runToPosition(1);

                //3. Strafe Right.

                setTargetPosition(STRAFE_RIGHT,2700);

                runToPosition(1);

                //3. Wobble Goal arm.

               setTargetPosition(WOBBLE_ARM_DOWN, 3100);

               runToPositionForWobbleArm(0.75);

               hardwarePushBot.wobbleGoalFinger.setPosition(0);

               //4. Wobble Goal Arm up
                setTargetPosition(WOBBLE_ARM_UP, 3100);

                runToPositionForWobbleArm(0.75);

                //5. Turn on shooting wheel.
                hardwarePushBot.shootingWheel.setPower(1);

                runWithoutEncoder();

                //6. Go back 0.5 using feedback
                drivewWithFeedback_FBM_Colors(0.5, 0, 1.0, "RED");

                //7. Strafe left using feedback
                drivewWithFeedback_FBM(0, -0.5, 1.0);

                drivewWithFeedback_FBM(-0.5, 0, 0.25);

                //8.Shooting Rings
                for (int i=0; i < 3; i++) {
                    hardwarePushBot.shootingTrigger.setPosition(1);
                    sleep(500);
                    hardwarePushBot.shootingTrigger.setPosition(0);
                    sleep(500);
                }

                //9. Move forward to park.
                //6. Go back 0.5 using feedback
                drivewWithFeedback_FBM(-0.5, 0, 0.25);
                break;
            case 2 :

                //1. Go to the right

                stopResetEncoder();

                setTargetPosition(STRAFE_LEFT,1000);

                runToPosition(1);

                //2. Go forward.

                setTargetPosition(DRIVE_FORWARD,4800);

                runToPosition(1);

                //3. Strafe right

                setTargetPosition(STRAFE_LEFT,1000);

                runToPosition(1);

                //4. Wobble Goal arm.

                setTargetPosition(WOBBLE_ARM_DOWN, 3100);

                runToPositionForWobbleArm(0.75);

                hardwarePushBot.wobbleGoalFinger.setPosition(0);

                break;
            case 3:

                //1. Go to the right

                stopResetEncoder();

                setTargetPosition(STRAFE_LEFT,1000);

                runToPosition(1);

                //2. Go forward.

                setTargetPosition(DRIVE_FORWARD,6300);

                runToPosition(1);

                //3. Strafe Right

                setTargetPosition(STRAFE_RIGHT,2700);

                runToPosition(1);

                //4. Wobble Goal arm.

                setTargetPosition(WOBBLE_ARM_DOWN, 3100);

                runToPositionForWobbleArm(0.75);

                hardwarePushBot.wobbleGoalFinger.setPosition(0);

                //4. Wobble Goal Arm up
                setTargetPosition(WOBBLE_ARM_UP, 3100);

                runToPositionForWobbleArm(0.75);

                setTargetPosition(DRIVE_BACK,2800);

                runToPosition(1);

                //5. Turn on shooting wheel.
                hardwarePushBot.shootingWheel.setPower(1);

                runWithoutEncoder();

                //6. Go back 0.5 using feedback
                drivewWithFeedback_FBM_Colors(0.5, 0, 1.0, "RED");

                //7. Strafe left using feedback
                drivewWithFeedback_FBM(0, -0.5, 1.0);

                drivewWithFeedback_FBM(-0.5, 0, 0.25);

                //8.Shooting Rings
                for (int i=0; i < 3; i++) {
                    hardwarePushBot.shootingTrigger.setPosition(1);
                    sleep(500);
                    hardwarePushBot.shootingTrigger.setPosition(0);
                    sleep(500);
                }

                //9. Move forward to park.
                //6. Go back 0.5 using feedback
                drivewWithFeedback_FBM(-0.5, 0, 0.25);
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
            tfod.setZoom(2.5, 16.0/9.0);
        }
    }

    /**
     * Ring Identification.
     */
    public void identifyRings() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() == 0 ) {
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

    /**
     * Drive/Strafe with feedback.
     * @param drive_power
     * @param strafe_power
     * @param timeOut
     */
    public void drivewWithFeedback_FBM(double drive_power, double strafe_power, double timeOut){
        hardwarePushBot.mecanumDrive(drive_power,strafe_power,0.0); // pass the parameters to a mecanumDrive method

        elapsedTime.reset();
        integralError=0;
        while (opModeIsActive()&& elapsedTime.seconds()<timeOut){
            heading = hardwarePushBot.getAngle();
            error =heading - 0; // desrired - current heading is the error
            integralError = integralError + error*0.025;

            hardwarePushBot.mecanumDrive(drive_power,strafe_power,-(error*GAIN_PROP+integralError*GAIN_INT)); // the multiplication of 0.015 is a gain to make the turn power small (not close to 1, which is maximum)
        }
        hardwarePushBot.mecanumDrive(0,0.0,0.0);
    }

    /**
     * Drive/Strafe with feedback.
     * @param drive_power
     * @param strafe_power
     */
    public void drivewWithFeedback_FBMEncoder(double drive_power, double strafe_power){
        hardwarePushBot.mecanumDrive(drive_power,strafe_power,0.0); // pass the parameters to a mecanumDrive method

        elapsedTime.reset();
        integralError=0;
        while (opModeIsActive()){
            heading = hardwarePushBot.getAngle();
            error =heading - 0; // desrired - current heading is the error
            integralError = integralError + error*0.025;

            hardwarePushBot.mecanumDrive(drive_power,strafe_power,-(error*GAIN_PROP+integralError*GAIN_INT)); // the multiplication of 0.015 is a gain to make the turn power small (not close to 1, which is maximum)
        }
        hardwarePushBot.mecanumDrive(0,0.0,0.0);
    }


    /**
     * Drive/Strafe with feedback loop and color detection.
     * @param drive_power
     * @param strafe_power
     * @param timeOut
     * @param colorString
     */

    public void drivewWithFeedback_FBM_Colors(double drive_power, double strafe_power, double timeOut, String colorString){
        hardwarePushBot.mecanumDrive(drive_power,strafe_power,0.0); // pass the parameters to a mecanumDrive method
        elapsedTime.reset();
        integralError=0;
        while (opModeIsActive()&& elapsedTime.seconds()<timeOut && !isColorFound(colorString)){

            heading = hardwarePushBot.getAngle();
            error = heading - 0; // desrired - current heading is the error
            integralError = integralError + error*0.025;

            hardwarePushBot.mecanumDrive(drive_power,strafe_power,-(error*GAIN_PROP+integralError*GAIN_INT)); // the multiplication of 0.015 is a gain to make the turn power small (not close to 1, which is maximum)
        }
        hardwarePushBot.mecanumDrive(0,0.0,0.0);
    }

    public boolean isColorFound(String colorString){
        switch(colorString) {
            case "RED":{
                return isRedColorFound();
            }
            case "WHITE":{
                return isWhitefound();
            }
            default:
                return false;
            //Add a case for blue
        }

    }

    /**
     * RED Color detection.
     * @return
     */
    public boolean isRedColorFound() {
        boolean found = false;

        Color.RGBToHSV((int) (hardwarePushBot.frontColorSensor.red() * SCALE_FACTOR),
                (int) (hardwarePushBot.frontColorSensor.green() * SCALE_FACTOR),
                (int) (hardwarePushBot.frontColorSensor.blue() * SCALE_FACTOR),
                hsvValues);
        // CheckForRed is a hsv value check

        hue = hsvValues[0];
        saturation = hsvValues[1];
        value = hsvValues[2];

        telemetry.addData("Color Red", String.valueOf(hue), String.valueOf(saturation), String.valueOf(value));
        telemetry.update();

        if((hue < 60 || hue > 320) ){
            found = true;
            sleep(50);
        }

        return found;
    }

    /**
     * White color detection.
     * @return
     */
    public boolean isWhitefound() {
        boolean found = false;
        double lightIntensity;
        //     lightIntensity = hardwarePushBot.rightColorSensor.alpha(); // total light luminosity


        Color.RGBToHSV((int) (hardwarePushBot.frontColorSensor.red() * SCALE_FACTOR),
                (int) (hardwarePushBot.frontColorSensor.green() * SCALE_FACTOR),
                (int) (hardwarePushBot.frontColorSensor.blue() * SCALE_FACTOR),
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
}
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="Encoder-qual2-Detect Red Right Target Zones")
public class RightRedTargetZones_Encoder_qual2 extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

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
    static int TURN_RIGHT = 6;
    static int TURN_LEFT = 7;
    static double SHOOTING_WHEEL_VELOCITY = -1760;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardwareMap();
        waitForStart();

        int counter = 1;
       while (opModeIsActive() && counter == 1) {
            identifyRings();
            goToTargetZones();
            deactivateTfod();
            counter++;
        }


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

        // Note changes for Strafer Chassis below
        hardwarePushBot.setWheelDirection();

        hardwarePushBot.wobbleGoalFinger.setPosition(1);
        hardwarePushBot.wobbleGoalFinger2.setPosition(1);
        hardwarePushBot.shootingTrigger.setPosition(0);
        initVuforia();
        initTfod();
        activateTfod();
        sleep(6500);

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
        } else if (action == 6) { //Turn Right
            hardwarePushBot.leftFrontWheel.setTargetPosition(hardwarePushBot.leftFrontWheel.getCurrentPosition() - position);
            hardwarePushBot.leftBackWheel.setTargetPosition(hardwarePushBot.leftBackWheel.getCurrentPosition() - position);
            hardwarePushBot.rightFrontWheel.setTargetPosition(hardwarePushBot.rightFrontWheel.getCurrentPosition() + position);
            hardwarePushBot.rightBackWheel.setTargetPosition(hardwarePushBot.rightBackWheel.getCurrentPosition() + position);
        } else if (action == 7) { //Turn Left
            hardwarePushBot.leftFrontWheel.setTargetPosition(hardwarePushBot.leftFrontWheel.getCurrentPosition() + position);
            hardwarePushBot.leftBackWheel.setTargetPosition(hardwarePushBot.leftBackWheel.getCurrentPosition() + position);
            hardwarePushBot.rightFrontWheel.setTargetPosition(hardwarePushBot.rightFrontWheel.getCurrentPosition() - position);
            hardwarePushBot.rightBackWheel.setTargetPosition(hardwarePushBot.rightBackWheel.getCurrentPosition() - position);
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

        while (opModeIsActive() && (hardwarePushBot.leftFrontWheel.isBusy() ||
                hardwarePushBot.rightFrontWheel.isBusy() && hardwarePushBot.leftBackWheel.isBusy() &&
                hardwarePushBot.rightBackWheel.isBusy())) {
            telemetry.addData("Current position right front", hardwarePushBot.rightFrontWheel.getCurrentPosition());
            telemetry.update();
        }
       hardwarePushBot.setWheelPower(0.0, 0.0, 0.0, 0.0);

    }

    public void runToPositionForWobbleArm(double power) {
        hardwarePushBot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hardwarePushBot.wobbleGoalArm.setPower(power);

        while (opModeIsActive() && hardwarePushBot.wobbleGoalArm.isBusy()) {
            telemetry.addData("Current position wobble arm", hardwarePushBot.wobbleGoalArm.getCurrentPosition());
            telemetry.update();
        }

        hardwarePushBot.wobbleGoalArm.setPower(0);

    }

    public void startShootingWheelByVoltage() {
        double battVoltage= getVoltage();
        hardwarePushBot.shootingWheel.setPower(0.748-(battVoltage-12)*(0.748-0.625)/(1.95));
    }

    public void startShootingWheelUsingEncoderbk() {
        hardwarePushBot.shootingWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardwarePushBot.shootingWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      //  hardwarePushBot.shootingWheel.setVelocity(SHOOTING_WHEEL_VELOCITY);
        hardwarePushBot.shootingWheel.setTargetPosition(-1000 + hardwarePushBot.shootingWheel.getCurrentPosition());
        hardwarePushBot.shootingWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardwarePushBot.shootingWheel.setPower(0.5);

        while (opModeIsActive() &&  hardwarePushBot.shootingWheel.isBusy()) {
            telemetry.addData("Current position wobble arm",  hardwarePushBot.shootingWheel.getCurrentPosition());
            telemetry.update();
        }

        hardwarePushBot.shootingWheel.setPower(0);
    }

    public void startShootingWheelUsingEncoder() {
        hardwarePushBot.shootingWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardwarePushBot.shootingWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardwarePushBot.shootingWheel.setVelocity(SHOOTING_WHEEL_VELOCITY);

    }

    public void goToTargetZones() {
        switch (targetZone) {
            case 1: //Target zone A
               stopResetEncoder();

               startShootingWheelUsingEncoder();  //Adjust shooting wheel based on battery voltage before starting to shoot.

               setTargetPosition(DRIVE_FORWARD,3275); //Drive forward using encoder counts.
               runToPosition(0.856);

                  //Shoot the rings one by one.
               for (int i=0; i < 3 && opModeIsActive(); i++) {
                    hardwarePushBot.shootingTrigger.setPosition(1);
                    sleep(550);
                    hardwarePushBot.shootingTrigger.setPosition(0);
                    sleep(500);
                }
                hardwarePushBot.shootingWheel.setPower(0); //turn off the shooting wheel*/

               runWithoutEncoder();
               drivewWithFeedback_FBM(0, 0.5, 1.15); //Strafe Right.
               drivewWithFeedback_FBM(-0.5, 0, 0.15);  // Adjust to go forward to get ready for shooting

               stopResetEncoder();

               setTargetPosition(WOBBLE_ARM_DOWN, 3100);  //Move Wobblearm down using encoder counts.
               runToPositionForWobbleArm(1.0);

               hardwarePushBot.wobbleGoalFinger.setPosition(0);  // Open the finger to release the wobble goal.
               hardwarePushBot.wobbleGoalFinger2.setPosition(0);
               sleep(250);

               setTargetPosition(WOBBLE_ARM_UP, 3100);  // Move wobble goal arm up using encoder counts.
               runToPositionForWobbleArm(1.0);

                setTargetPosition(TURN_LEFT,2290);  // Turn left 45 degrees to get second wobble goal using enocder.
                runToPosition(1.0);

                setTargetPosition(WOBBLE_ARM_DOWN, 3100); // Move the wobble arm down using encoder.
                runToPositionForWobbleArm(0.75);

                hardwarePushBot.wobbleGoalFinger.setPosition(0);  // Open the finger to get ready to pick up.
                hardwarePushBot.wobbleGoalFinger2.setPosition(0);

                setTargetPosition(DRIVE_FORWARD,2300);  // Drive forward towards second wobble goal using encoder.
                runToPosition(1.0);

                hardwarePushBot.wobbleGoalFinger.setPosition(1.0);  // Close the finger.
                hardwarePushBot.wobbleGoalFinger2.setPosition(1.0);
                sleep(200);

                setTargetPosition(WOBBLE_ARM_UP, 1000);  // Move the warm slightly up to avoid wobble arm damage.
                runToPositionForWobbleArm(1.0);

                setTargetPosition(DRIVE_BACK,2050);  // Drive backwards towards target zone.
                runToPosition(1.0);

                setTargetPosition(TURN_RIGHT,2000);  // Turn back 45 degrees to go to target zone
                runToPosition(1.0);

                setTargetPosition(WOBBLE_ARM_DOWN, 1000);  // Move the warm slightly up to avoid wobble arm damage.
                runToPositionForWobbleArm(1.0);

                hardwarePushBot.wobbleGoalFinger.setPosition(0);  // Release the finger
                hardwarePushBot.wobbleGoalFinger2.setPosition(0);

                setTargetPosition(WOBBLE_ARM_UP, 1000);  // Move the warm slightly up to avoid wobble arm damage.
                runToPositionForWobbleArm(1.0);

                runWithoutEncoder();

                drivewWithFeedback_FBM(0.5, 0, 0.25); //Slightly move forward to adjust and get ready fo

                drivewWithFeedback_FBM(0, -0.5, 1.5); // Strafe left based on time.

                drivewWithFeedback_FBM(-0.5, 0, 1.0); //Slightly move forward to adjust and get ready fo

                break;
            case 2 : // Target zone B

                stopResetEncoder();

                setTargetPosition(STRAFE_RIGHT,1325);  // Strafe right
                runToPosition(1);

                startShootingWheelUsingEncoder();  //Start shooting wheel using encoder.

                setTargetPosition(DRIVE_FORWARD,3275);  // Drive forward.
                runToPosition(1);

                setTargetPosition(STRAFE_LEFT,1325); // Strafe Left.
                runToPosition(0.85);

                //Shoot the rings one by one.
                for (int i=0; i < 3 && opModeIsActive(); i++) {
                    hardwarePushBot.shootingTrigger.setPosition(1);
                    sleep(550);
                    hardwarePushBot.shootingTrigger.setPosition(0);
                    sleep(500);
                }
                hardwarePushBot.shootingWheel.setPower(0); //turn off the shooting wheel*/

                runWithoutEncoder();
                drivewWithFeedback_FBM(0, -0.5, 0.40); //Strafe Left.

                stopResetEncoder();
                setTargetPosition(DRIVE_FORWARD,1700);  // Drive forward.
                runToPosition(1);

                setTargetPosition(WOBBLE_ARM_DOWN, 3100);  //Move wobble arm down to place thw wobble goal.
                runToPositionForWobbleArm(0.75);

                hardwarePushBot.wobbleGoalFinger.setPosition(0);  //Release the finger.
                hardwarePushBot.wobbleGoalFinger2.setPosition(0);
                sleep(100);

                setTargetPosition(WOBBLE_ARM_UP, 3100);  //Move the wobble arm up.
                runToPositionForWobbleArm(0.75);

                runWithoutEncoder();
                drivewWithFeedback_FBM(0.5, 0, 0.55);  // Adjust to go backward
                drivewWithFeedback_FBM(0, -0.5, 0.40); //Strafe Left to get ready for next

                stopResetEncoder();
                setTargetPosition(TURN_LEFT,2700);  // Turn around 180 degrees to get second wobble goal using enocder.
                runToPosition(1.0);

                setTargetPosition(WOBBLE_ARM_DOWN, 3100);  //Move wobble arm down to place thw wobble goal.
                runToPositionForWobbleArm(0.75);

                hardwarePushBot.wobbleGoalFinger.setPosition(0);  //Release the finger.
                hardwarePushBot.wobbleGoalFinger2.setPosition(0);

                setTargetPosition(DRIVE_FORWARD,2600);  // Turn around 180 degrees to get second wobble goal using enocder.
                runToPosition(1.0);

                hardwarePushBot.wobbleGoalFinger.setPosition(1);  //Close the finger.
                hardwarePushBot.wobbleGoalFinger2.setPosition(1);
                sleep(100);

                setTargetPosition(WOBBLE_ARM_UP, 1000);  // Move the warm slightly up to avoid wobble arm damage.
                runToPositionForWobbleArm(1.0);

                setTargetPosition(DRIVE_BACK,2575);  // Turn around 180 degrees to get second wobble goal using enocder.
                runToPosition(1.0);

                setTargetPosition(TURN_LEFT,2700);  // Turn around 180 degrees to get second wobble goal using enocder.
                runToPosition(1.0);

                setTargetPosition(WOBBLE_ARM_DOWN, 1000);  // Move the warm slightly up to avoid wobble arm damage.
                runToPositionForWobbleArm(1.0);

                hardwarePushBot.wobbleGoalFinger.setPosition(0);  //Release the finger.
                hardwarePushBot.wobbleGoalFinger2.setPosition(0);

                setTargetPosition(WOBBLE_ARM_UP, 1000);  // Move the warm slightly up to avoid wobble arm damage.
                runToPositionForWobbleArm(1.0);

                break;
            case 3:  // Target zone C

                stopResetEncoder(); //Strafe Right
                setTargetPosition(STRAFE_RIGHT,1325);
                runToPosition(1);

                startShootingWheelUsingEncoder();  //Start shooting wheel using encoder.

                setTargetPosition(DRIVE_FORWARD,3275);  // Drive forward.
                runToPosition(1);

                setTargetPosition(STRAFE_LEFT,1325); // Strafe Left.
                runToPosition(0.85);

                //Shoot the rings one by one.
                for (int i=0; i < 3 && opModeIsActive(); i++) {
                    hardwarePushBot.shootingTrigger.setPosition(1);
                    sleep(550);
                    hardwarePushBot.shootingTrigger.setPosition(0);
                    sleep(500);
                }
                hardwarePushBot.shootingWheel.setPower(0); //turn off the shooting wheel*/

                setTargetPosition(TURN_RIGHT,1050);  // Turn around 15 degrees using enocder.
                runToPosition(1.0);

                setTargetPosition(DRIVE_FORWARD,1900);  // Drive forward.
                runToPosition(1);

                setTargetPosition(WOBBLE_ARM_DOWN, 3100); //Move wobble arm down.
                runToPositionForWobbleArm(0.75);

                hardwarePushBot.wobbleGoalFinger.setPosition(0);  //Release finger to drop wobble goal.
                hardwarePushBot.wobbleGoalFinger2.setPosition(0);
                sleep(150);

                setTargetPosition(WOBBLE_ARM_UP, 3100); //Wobble arm up.
                runToPositionForWobbleArm(0.75);

             /*   setTargetPosition(DRIVE_BACK,2800);  //Drive back.
                runToPosition(1);

                startShootingWheelByVoltage(); //Turn on the shooting wheel

                runWithoutEncoder();

                drivewWithFeedback_FBM_Colors(0.5, 0, 1.0, "RED"); //Go back until Red.

                drivewWithFeedback_FBM(0, -0.5, 1.0); //Strafe Left.

                drivewWithFeedback_FBM(-0.5, 0, 0.25);  //Drive foward to adjust.

                startShootingWheelByVoltage();  //Adjust shooting wheel based on battery voltage before starting to shoot.
                sleep(100);

                for (int i=0; i < 3 && opModeIsActive(); i++) {  //Shooting rings.
                    hardwarePushBot.shootingTrigger.setPosition(1);
                    sleep(500);
                    hardwarePushBot.shootingTrigger.setPosition(0);
                    sleep(500);
                }

                drivewWithFeedback_FBM(-0.5, 0, 0.45); //Go forward to park in the launch line
*/
                break;
        }
    }

    public void goToTargetZonesbk() {
        switch (targetZone) {
            case 1: //Target zone A
                stopResetEncoder(); //Strafe Right using encoder counts.
                setTargetPosition(STRAFE_RIGHT,1320);
                runToPosition(1);

                setTargetPosition(DRIVE_FORWARD,3800); //Drive forward using encoder counts.
                runToPosition(1);

                setTargetPosition(WOBBLE_ARM_DOWN, 3100);  //Move Wobblearm down using encoder counts.
                runToPositionForWobbleArm(1.0);

                hardwarePushBot.wobbleGoalFinger.setPosition(0);  // Open the finger to release the wobble goal.
                hardwarePushBot.wobbleGoalFinger2.setPosition(0);
                sleep(250);

                setTargetPosition(WOBBLE_ARM_UP, 3100);  // Move wobble goal arm up using encoder counts.
                runToPositionForWobbleArm(1.0);

                startShootingWheelByVoltage();  // Start the shooting wheel.

                runWithoutEncoder();  //Do not use encoders from  now.

                drivewWithFeedback_FBM_Colors(0.5, 0, 1.0, "RED");  // Go backward until red color.

                drivewWithFeedback_FBM(0, -0.35, 1.7); // Strafe left based on time.

                drivewWithFeedback_FBM(-0.5, 0, 0.15); //Slightly move forward to adjust and get ready for shooting rings.

                startShootingWheelByVoltage();  //Adjust shooting wheel based on battery voltage before starting to shoot.
                sleep(500);

                //Shoot the rings one by one.
                for (int i=0; i < 3 && opModeIsActive(); i++) {
                    hardwarePushBot.shootingTrigger.setPosition(1);
                    sleep(500);
                    hardwarePushBot.shootingTrigger.setPosition(0);
                    sleep(500);
                }
                hardwarePushBot.shootingWheel.setPower(0); //turn off the shooting wheel

                stopResetEncoder();  //Start the encoder.

                setTargetPosition(STRAFE_LEFT,1175);  //Strafe left using encoder counts.
                runToPosition(1.0);

                setTargetPosition(TURN_RIGHT,2700);  // Turn around 180 degrees to get second wobble goal using enocder.
                runToPosition(1.0);

                setTargetPosition(WOBBLE_ARM_DOWN, 3100); // Move the wobble arm down using encoder.
                runToPositionForWobbleArm(0.75);

                hardwarePushBot.wobbleGoalFinger.setPosition(0);  // Open the finger to get ready to pick up.
                hardwarePushBot.wobbleGoalFinger2.setPosition(0);

                setTargetPosition(DRIVE_FORWARD,1750);  // Drive forward towards second wobble goal using encoder.
                runToPosition(1.0);

                hardwarePushBot.wobbleGoalFinger.setPosition(1.0);  // Close the finger.
                hardwarePushBot.wobbleGoalFinger2.setPosition(1.0);
                sleep(200);

                setTargetPosition(WOBBLE_ARM_UP, 1000);  // Move the warm slightly up to avoid wobble arm damage.
                runToPositionForWobbleArm(1.0);

                setTargetPosition(TURN_RIGHT,2700);  // Turn around 180 degrees to go to target zone
                runToPosition(1.0);

                setTargetPosition(DRIVE_FORWARD,2350);  // Drive forward
                runToPosition(1.0);

                setTargetPosition(STRAFE_RIGHT,1700);  // Strafe right to target zone A
                runToPosition(1.0);

                setTargetPosition(WOBBLE_ARM_DOWN, 1000);  // Move the warm slightly up to avoid wobble arm damage.
                runToPositionForWobbleArm(1.0);

                hardwarePushBot.wobbleGoalFinger.setPosition(0);  // Release the finger
                hardwarePushBot.wobbleGoalFinger2.setPosition(0);

                setTargetPosition(WOBBLE_ARM_UP, 1000);  // Move the warm slightly up to avoid wobble arm damage.
                runToPositionForWobbleArm(1.0);

                runWithoutEncoder();

                drivewWithFeedback_FBM(0, -0.5, 0.75); // Strafe left based on time.

                drivewWithFeedback_FBM(-0.5, 0, 0.25); //Slightly move forward to adjust and get ready fo

                break;
            case 2 : // Target zone B

                stopResetEncoder(); // Strafe Right.
                setTargetPosition(STRAFE_RIGHT,1325);
                runToPosition(1);

                setTargetPosition(DRIVE_FORWARD,5000);  // Drive forward.
                runToPosition(1);

                setTargetPosition(STRAFE_LEFT,1600); // Strafe Left.
                runToPosition(1);

                setTargetPosition(WOBBLE_ARM_DOWN, 3100);  //Move wobble arm down to place thw wobble goal.
                runToPositionForWobbleArm(0.75);

                hardwarePushBot.wobbleGoalFinger.setPosition(0);  //Release the finger.
                hardwarePushBot.wobbleGoalFinger2.setPosition(0);
                sleep(100);

                setTargetPosition(WOBBLE_ARM_UP, 3100);  //Move the wobble arm up.
                runToPositionForWobbleArm(0.75);

                setTargetPosition(STRAFE_RIGHT,1600);  //Strafe right
                runToPosition(1);

                runWithoutEncoder();

                drivewWithFeedback_FBM_Colors(0.5, 0, 3.0, "RED"); // Go back until red color.

                drivewWithFeedback_FBM_Colors(0.5, 0, 2.0, "RED");  // Go back until red color.

                startShootingWheelByVoltage(); //Start shooting wheel based on battery voltage sensor.

                drivewWithFeedback_FBM(0, -0.5, 1.1);  //Strafe left
                drivewWithFeedback_FBM(-0.5, 0, 0.15);  // Adjust to go forward to get ready for shooting

                startShootingWheelByVoltage(); //Start shooting wheel based on battery voltage sensor.
                sleep(100);

                for (int i=0; i < 3 && opModeIsActive(); i++) {  //Start shooting rings
                    hardwarePushBot.shootingTrigger.setPosition(1);
                    sleep(500);
                    hardwarePushBot.shootingTrigger.setPosition(0);
                    sleep(500);
                }

                drivewWithFeedback_FBM(-0.5, 0, 0.45);  // Park launch line.
                break;
            case 3:  // Target zone C

                stopResetEncoder(); //Strafe Right
                setTargetPosition(STRAFE_RIGHT,1325);
                runToPosition(1);

                setTargetPosition(DRIVE_FORWARD,6550); //Go forward to target zone c.
                runToPosition(1);

                setTargetPosition(WOBBLE_ARM_DOWN, 3100); //Move wobble arm down.
                runToPositionForWobbleArm(0.75);

                hardwarePushBot.wobbleGoalFinger.setPosition(0);  //Release finger to drop wobble goal.
                hardwarePushBot.wobbleGoalFinger2.setPosition(0);
                sleep(150);

                setTargetPosition(WOBBLE_ARM_UP, 3100); //Wobble arm up.
                runToPositionForWobbleArm(0.75);

                setTargetPosition(DRIVE_BACK,2800);  //Drive back.
                runToPosition(1);

                startShootingWheelByVoltage(); //Turn on the shooting wheel

                runWithoutEncoder();

                drivewWithFeedback_FBM_Colors(0.5, 0, 1.0, "RED"); //Go back until Red.

                drivewWithFeedback_FBM(0, -0.5, 1.0); //Strafe Left.

                drivewWithFeedback_FBM(-0.5, 0, 0.25);  //Drive foward to adjust.

                startShootingWheelByVoltage();  //Adjust shooting wheel based on battery voltage before starting to shoot.
                sleep(100);

                for (int i=0; i < 3 && opModeIsActive(); i++) {  //Shooting rings.
                    hardwarePushBot.shootingTrigger.setPosition(1);
                    sleep(500);
                    hardwarePushBot.shootingTrigger.setPosition(0);
                    sleep(500);
                }

                drivewWithFeedback_FBM(-0.5, 0, 0.45); //Go forward to park in the launch line

                break;
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
        while (opModeIsActive() && elapsedTime.seconds()<timeOut){
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
        while (opModeIsActive() && elapsedTime.seconds()<timeOut && !isColorFound(colorString)){
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
              hue = hsvValues[0];
        saturation = hsvValues[1];
        value = hsvValues[2];
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

    /**
     * `alize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam1");
        parameters.useExtendedTracking = true;
        parameters.cameraMonitorFeedback= VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f;
        tfodParameters.useObjectTracker = true;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

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

    /**
     * Ring Identification.
     */
    public void identifyRings() {
        if (opModeIsActive()) {
           // List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            List<Recognition> updatedRecognitions = tfod.getRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() == 0) {
                    // empty list.  no objects recognized.
                    telemetry.addData("TFOD", "No items detected.");
                    telemetry.addData("Target Zone", "A");
                    this.targetZone = 1;
                } else {
                    // list is not empty.
                    // step through the list of recognitions and display boundary info.
                  //  int i = 0;
                    for (int i=0; i < updatedRecognitions.size() && opModeIsActive(); i++) {
                        Recognition recognition = updatedRecognitions.get(i);
                  //  for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());

                            // check label to see which target zone to go after.
                            if (recognition.getLabel().equals("Single")) {
                                telemetry.addData("Target Zone", "B");
                                this.targetZone = 2;

                            } else if (recognition.getLabel().equals("Quad")) {
                                telemetry.addData("Target Zone", "C");
                                this.targetZone = 3;

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
}
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
    static int DRIVE_BACK = 0;
    static int DRIVE_FORWARD = 1;
    static int STRAFE_RIGHT = 2;
    static int STRAFE_LEFT = 3;
    static int WOBBLE_ARM_DOWN = 4;
    static int WOBBLE_ARM_UP = 5;
    static int TURN_RIGHT = 6;
    static int TURN_LEFT = 7;
    static int DIAG_RIGHT_FRONT = 8;
    static int DIAG_LEFT_FRONT = 9;
    static int DIAG_LEFT_BACK = 10;
    static int DIAG_RIGHT_BACK = 11;
    static double SHOOTING_WHEEL_VELOCITY = -1750;
    int leftFrontPosition=0;
    int  rightFrontPosition=0;
    int  leftBackPosition=0;
    int  rightBackPosition=0;

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
        hardwarePushBot.mapTouchSensor(hardwareMap);

        hardwarePushBot.touchSensorWaFront.setMode(DigitalChannel.Mode.INPUT);


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

    public void setTargetPosition_FBM(int action, int position) {
        if (action == 0) { // Drive backward
            leftFrontPosition = hardwarePushBot.leftFrontWheel.getCurrentPosition() + position;
            leftBackPosition = hardwarePushBot.leftBackWheel.getCurrentPosition() + position;
            rightFrontPosition = hardwarePushBot.rightFrontWheel.getCurrentPosition() + position;
            rightBackPosition = hardwarePushBot.rightBackWheel.getCurrentPosition() + position;
            hardwarePushBot.leftFrontWheel.setTargetPosition(leftFrontPosition);
            hardwarePushBot.leftBackWheel.setTargetPosition(leftBackPosition);
            hardwarePushBot.rightFrontWheel.setTargetPosition(rightFrontPosition);
            hardwarePushBot.rightBackWheel.setTargetPosition(rightBackPosition);
        } else if (action == 1) { // Drive forward
            leftFrontPosition = hardwarePushBot.leftFrontWheel.getCurrentPosition() - position;
            leftBackPosition = hardwarePushBot.leftBackWheel.getCurrentPosition() - position;
            rightFrontPosition = hardwarePushBot.rightFrontWheel.getCurrentPosition() - position;
            rightBackPosition = hardwarePushBot.rightBackWheel.getCurrentPosition() - position;
            hardwarePushBot.leftFrontWheel.setTargetPosition(leftFrontPosition);
            hardwarePushBot.leftBackWheel.setTargetPosition(leftBackPosition);
            hardwarePushBot.rightFrontWheel.setTargetPosition(rightFrontPosition);
            hardwarePushBot.rightBackWheel.setTargetPosition(rightBackPosition);
        } else if (action == 2) { // Strafe right
            leftFrontPosition = hardwarePushBot.leftFrontWheel.getCurrentPosition() - position;
            leftBackPosition = hardwarePushBot.leftBackWheel.getCurrentPosition() + position;
            rightFrontPosition = hardwarePushBot.rightFrontWheel.getCurrentPosition() + position;
            rightBackPosition = hardwarePushBot.rightBackWheel.getCurrentPosition() - position;
            hardwarePushBot.leftFrontWheel.setTargetPosition(leftFrontPosition);
            hardwarePushBot.leftBackWheel.setTargetPosition(leftBackPosition);
            hardwarePushBot.rightFrontWheel.setTargetPosition(rightFrontPosition);
            hardwarePushBot.rightBackWheel.setTargetPosition(rightBackPosition);
        }  else if (action == 3) { // Strafe left
            leftFrontPosition = hardwarePushBot.leftFrontWheel.getCurrentPosition() + position;
            leftBackPosition = hardwarePushBot.leftBackWheel.getCurrentPosition() - position;
            rightFrontPosition = hardwarePushBot.rightFrontWheel.getCurrentPosition() - position;
            rightBackPosition = hardwarePushBot.rightBackWheel.getCurrentPosition() + position;
            hardwarePushBot.leftFrontWheel.setTargetPosition(leftFrontPosition);
            hardwarePushBot.leftBackWheel.setTargetPosition(leftBackPosition);
            hardwarePushBot.rightFrontWheel.setTargetPosition(rightFrontPosition);
            hardwarePushBot.rightBackWheel.setTargetPosition(rightBackPosition);
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
        } else if (action == 8) { //diagonal right front
            hardwarePushBot.leftFrontWheel.setTargetPosition(hardwarePushBot.leftFrontWheel.getCurrentPosition() - position);
            hardwarePushBot.rightBackWheel.setTargetPosition(hardwarePushBot.rightBackWheel.getCurrentPosition() - position);
        }else if (action == 9) { //diagonal left front
            hardwarePushBot.leftBackWheel.setTargetPosition(hardwarePushBot.leftBackWheel.getCurrentPosition() - position);
            hardwarePushBot.rightFrontWheel.setTargetPosition(hardwarePushBot.rightFrontWheel.getCurrentPosition() - position);
        }else if (action == 10) { //diagonal left back
            hardwarePushBot.leftFrontWheel.setTargetPosition(hardwarePushBot.leftFrontWheel.getCurrentPosition() + position);
            hardwarePushBot.rightBackWheel.setTargetPosition(hardwarePushBot.rightBackWheel.getCurrentPosition() + position);
        }else if (action == 11) { //diagonal right back
            hardwarePushBot.leftBackWheel.setTargetPosition(hardwarePushBot.leftBackWheel.getCurrentPosition() + position);
            hardwarePushBot.rightFrontWheel.setTargetPosition(hardwarePushBot.rightFrontWheel.getCurrentPosition() + position);

        }

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
        } else if (action == 8) { //diagonal right front
            hardwarePushBot.leftFrontWheel.setTargetPosition(hardwarePushBot.leftFrontWheel.getCurrentPosition() - position);
            hardwarePushBot.rightBackWheel.setTargetPosition(hardwarePushBot.rightBackWheel.getCurrentPosition() - position);
        }else if (action == 9) { //diagonal left front
            hardwarePushBot.leftBackWheel.setTargetPosition(hardwarePushBot.leftBackWheel.getCurrentPosition() - position);
            hardwarePushBot.rightFrontWheel.setTargetPosition(hardwarePushBot.rightFrontWheel.getCurrentPosition() - position);
        }else if (action == 10) { //diagonal left back
            hardwarePushBot.leftFrontWheel.setTargetPosition(hardwarePushBot.leftFrontWheel.getCurrentPosition() + position);
            hardwarePushBot.rightBackWheel.setTargetPosition(hardwarePushBot.rightBackWheel.getCurrentPosition() + position);
        }else if (action == 11) { //diagonal right back
            hardwarePushBot.leftBackWheel.setTargetPosition(hardwarePushBot.leftBackWheel.getCurrentPosition() + position);
            hardwarePushBot.rightFrontWheel.setTargetPosition(hardwarePushBot.rightFrontWheel.getCurrentPosition() + position);

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

        hardwarePushBot.leftFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwarePushBot.rightFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwarePushBot.leftBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwarePushBot.rightBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hardwarePushBot.setWheelPower(power, power, power, power);

        while (opModeIsActive() && (hardwarePushBot.leftFrontWheel.isBusy() &&
                hardwarePushBot.rightFrontWheel.isBusy() && hardwarePushBot.leftBackWheel.isBusy() &&
                hardwarePushBot.rightBackWheel.isBusy())) {
            telemetry.addData("Current position right front", hardwarePushBot.rightFrontWheel.getCurrentPosition());
            telemetry.update();
        }
       hardwarePushBot.setWheelPower(0.0, 0.0, 0.0, 0.0);

    }

    public void runToPosition_FBM(double power) {
        runToPositionMode();

        hardwarePushBot.leftFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwarePushBot.rightFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwarePushBot.leftBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwarePushBot.rightBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        integralError=0;

        while (opModeIsActive() && hardwarePushBot.leftFrontWheel.getCurrentPosition() != leftFrontPosition && hardwarePushBot.leftBackWheel.getCurrentPosition() != leftBackPosition
                && hardwarePushBot.rightFrontWheel.getCurrentPosition() != rightFrontPosition && hardwarePushBot.rightBackWheel.getCurrentPosition() != rightBackPosition ) {

             heading = hardwarePushBot.getAngle();
             error =heading - 0; // desrired - current heading is the error
             integralError = integralError + error*0.025;
             hardwarePushBot.mecanumDrive(power,0,-(error*GAIN_PROP+integralError*GAIN_INT));
             telemetry.addData("Current position right front", hardwarePushBot.rightFrontWheel.getCurrentPosition());
             telemetry.update();
        }

      hardwarePushBot.setWheelPower(0.0, 0.0, 0.0, 0.0);


    }

    public void runToPositionForWobbleArm(double power) {
        hardwarePushBot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hardwarePushBot.wobbleGoalArm.setPower(power);

        while (opModeIsActive() && hardwarePushBot.wobbleGoalArm.isBusy() && hardwarePushBot.touchSensorWaFront.getState()) {
            telemetry.addData("Current position wobble arm", hardwarePushBot.wobbleGoalArm.getCurrentPosition());
            telemetry.update();
        }

        hardwarePushBot.wobbleGoalArm.setPower(0);

    }

    public void runToPositionForWobbleArmBack(double power) {
        hardwarePushBot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hardwarePushBot.wobbleGoalArm.setPower(power);

        while (opModeIsActive() && hardwarePushBot.wobbleGoalArm.isBusy() && hardwarePushBot.touchSensorWaBack.getState()) {
            telemetry.addData("Current position wobble arm", hardwarePushBot.wobbleGoalArm.getCurrentPosition());
            telemetry.update();
        }

        hardwarePushBot.wobbleGoalArm.setPower(0);

    }

    public void startShootingWheelByVoltage() {
        double battVoltage= getVoltage();
        hardwarePushBot.shootingWheel.setPower(0.748-(battVoltage-12)*(0.748-0.625)/(1.95));
    }

    public void startShootingWheelUsingEncoder() {
        hardwarePushBot.shootingWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardwarePushBot.shootingWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardwarePushBot.shootingWheel.setVelocity(SHOOTING_WHEEL_VELOCITY);

    }

    public void runToPositionDiag_LF_RB(){
        hardwarePushBot.leftFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwarePushBot.rightFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwarePushBot.leftBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwarePushBot.rightBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hardwarePushBot.leftFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardwarePushBot.leftBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardwarePushBot.rightFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardwarePushBot.rightBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //runToPositionAll(1, 0,0,1);
        hardwarePushBot.setWheelPower(1,0.0,0.0,1);

        while (opModeIsActive() && (hardwarePushBot.leftFrontWheel.isBusy() &&
                hardwarePushBot.rightBackWheel.isBusy()
        )) {
            telemetry.addData("Current position right front", hardwarePushBot.leftFrontWheel.getCurrentPosition());
            telemetry.update();
        }
        hardwarePushBot.setWheelPower(0.0, 0.0, 0.0, 0.0);
    }

    public void runToPositionDiag_RF_LB(){
        hardwarePushBot.leftFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwarePushBot.rightFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwarePushBot.leftBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwarePushBot.rightBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hardwarePushBot.leftFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardwarePushBot.leftBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardwarePushBot.rightFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardwarePushBot.rightBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //runToPositionAll(1, 0,0,1);
        hardwarePushBot.setWheelPower(0,1,1,0);

        while (opModeIsActive() && (hardwarePushBot.leftBackWheel.isBusy() &&
                hardwarePushBot.rightFrontWheel.isBusy()
        )) {
            telemetry.addData("Current position right front", hardwarePushBot.rightFrontWheel.getCurrentPosition());
            telemetry.update();
        }
        hardwarePushBot.setWheelPower(0.0, 0.0, 0.0, 0.0);
    }

    public void goToTargetZones() {
        switch (targetZone) {
            case 1: //Target zone A
                stopResetEncoder();

                setTargetPosition(DRIVE_FORWARD,1690); //Drive forward using encoder counts.
                runToPosition(0.75);

                runWithoutEncoder();

                //Go to the right
                drivewWithFeedback_FBM(0, 0.45, 0.95);

                stopResetEncoder();

                //Wobble Goal Arm Down
                setTargetPosition(WOBBLE_ARM_DOWN, 3100);
                runToPositionForWobbleArm(1.0);

                //Open Claw
                hardwarePushBot.wobbleGoalFinger.setPosition(0);
                hardwarePushBot.wobbleGoalFinger2.setPosition(0);

                sleep(200);

                runWithoutEncoder();

                //Drive Back
                drivewWithFeedback_FBM(0.5, 0, 0.30);

                stopResetEncoder();

                //Turn Left
                setTargetPosition(TURN_LEFT,1105);
                runToPosition(0.8);

                //Drive Forward
                setTargetPosition(DRIVE_FORWARD,985);
                runToPosition(0.8);

                //sleep(300);

                //Close Claw
                hardwarePushBot.wobbleGoalFinger.setPosition(1);
                hardwarePushBot.wobbleGoalFinger2.setPosition(1);
                sleep(500);

                //Wobble Goal Arm Up slightly
                setTargetPosition(WOBBLE_ARM_UP, 2000);
                runToPositionForWobbleArmBack(1.0);

                //Drive Back
                setTargetPosition(DRIVE_BACK,675);
                runToPosition(0.8);

                //Turn Right
                setTargetPosition(TURN_RIGHT,1200);
                runToPosition(0.8);

                //Drive Forward
                setTargetPosition(DRIVE_FORWARD,280);
                runToPosition(0.8);

                //Wobble Goal Arm Up slightly
                setTargetPosition(WOBBLE_ARM_DOWN, 1950);
                runToPositionForWobbleArm(1.0);

                hardwarePushBot.wobbleGoalFinger.setPosition(0);
                hardwarePushBot.wobbleGoalFinger2.setPosition(0);
                sleep(500);

                //Drive Back
                setTargetPosition(DRIVE_BACK,250);
                runToPosition(0.8);

                //Wobble Goal Arm Up slightly
                setTargetPosition(WOBBLE_ARM_UP, 2000);
                runToPositionForWobbleArmBack(1.0);

                SHOOTING_WHEEL_VELOCITY = -1700;
                startShootingWheelUsingEncoder(SHOOTING_WHEEL_VELOCITY);

                runWithoutEncoder();

                //Strafe Left
                drivewWithFeedback_FBM(0, -0.2, 0.75);

                setTargetPosition(STRAFE_LEFT,610);
                runToPosition(0.6);

                //drivewWithFeedback_FBM(0.6, 0, 0.25);

                hardwarePushBot.shootingTrigger.setPosition(1);
                sleep(550);
                hardwarePushBot.shootingTrigger.setPosition(0);
                sleep(500);

                stopResetEncoder();

                setTargetPosition(TURN_LEFT, 75);
                runToPosition(0.6);

                hardwarePushBot.shootingTrigger.setPosition(1);
                sleep(550);
                hardwarePushBot.shootingTrigger.setPosition(0);
                sleep(500);

                setTargetPosition(TURN_LEFT, 45);
                runToPosition(0.6);

                hardwarePushBot.shootingTrigger.setPosition(1);
                sleep(550);
                hardwarePushBot.shootingTrigger.setPosition(0);
                sleep(500);

                runWithoutEncoder();
                drivewWithFeedback_FBM(-0.6, 0, 0.45);

                break;
            case 2 : // Target zone B

                stopResetEncoder();

                setTargetPosition(DIAG_RIGHT_FRONT,1530);
                runToPositionDiag_LF_RB();

                startShootingWheelUsingEncoder();  //Start shooting wheel using encoder.

                setTargetPosition(DRIVE_FORWARD,600);
                runToPosition(0.8);

                runWithoutEncoder();
                drivewWithFeedback_FBM(0, -0.5, 0.75); //Strafe Left to get ready for shooting
                drivewWithFeedback_FBM(-0.5, 0, 0.2); //Slightly move forward to adjust and get ready fo
                stopResetEncoder();

                //Shoot the rings one by one.
                for (int i=0; i < 3 && opModeIsActive(); i++) {
                    hardwarePushBot.shootingTrigger.setPosition(1);
                    sleep(550);
                    hardwarePushBot.shootingTrigger.setPosition(0);
                    sleep(500);
                }
                hardwarePushBot.shootingWheel.setPower(0); //turn off the shooting wheel*/

                runWithoutEncoder();
                drivewWithFeedback_FBM(0, -0.5, 0.21); //Strafe Left.

                stopResetEncoder();
                setTargetPosition(DRIVE_FORWARD,675);  // Drive forward.
                runToPosition(0.8);

                setTargetPosition(WOBBLE_ARM_DOWN, 3100);  //Move wobble arm down to place thw wobble goal.
                runToPositionForWobbleArm(0.75);

                hardwarePushBot.wobbleGoalFinger.setPosition(0);  //Release the finger.
                hardwarePushBot.wobbleGoalFinger2.setPosition(0);
                sleep(100);

                setTargetPosition(WOBBLE_ARM_UP, 3100);  //Move the wobble arm up.
                runToPositionForWobbleArmBack(0.75);

                runWithoutEncoder();
                drivewWithFeedback_FBM(0.5, 0, 0.25);  // Adjust to go backward
                drivewWithFeedback_FBM(0, -0.5, 0.75); //Strafe Left to get ready for next

                stopResetEncoder();
                setTargetPosition(TURN_LEFT,1350);  // Turn around 180 degrees to get second wobble goal using enocder.
                runToPosition(0.9);

                setTargetPosition(WOBBLE_ARM_DOWN, 3100);  //Move wobble arm down to place thw wobble goal.
                runToPositionForWobbleArm(0.75);

                hardwarePushBot.wobbleGoalFinger.setPosition(0);  //Release the finger.
                hardwarePushBot.wobbleGoalFinger2.setPosition(0);

                setTargetPosition(DRIVE_FORWARD,1275);  // Turn around 180 degrees to get second wobble goal using enocder.
                runToPosition(0.85);

                hardwarePushBot.wobbleGoalFinger.setPosition(1);  //Close the finger.
                hardwarePushBot.wobbleGoalFinger2.setPosition(1);
                sleep(400);

                setTargetPosition(WOBBLE_ARM_UP, 2000);  // Move the warm slightly up to avoid wobble arm damage.
                runToPositionForWobbleArm(0.85);

                setTargetPosition(DRIVE_BACK,1275);  // Drive Back.
                runToPosition(0.85);

                setTargetPosition(TURN_LEFT,1275);  // Turn around 180 degrees to get second wobble goal using enocder.
                runToPosition(0.9);

                setTargetPosition(WOBBLE_ARM_DOWN, 3100);  // Move the warm slightly up to avoid wobble arm damage.
                runToPositionForWobbleArm(1.0);

                hardwarePushBot.wobbleGoalFinger.setPosition(0);  //Release the finger.
                hardwarePushBot.wobbleGoalFinger2.setPosition(0);
                sleep(200);

                setTargetPosition(WOBBLE_ARM_UP, 2000);  // Move the warm slightly up to avoid wobble arm damage.
                runToPositionForWobbleArm(1.0);

                runWithoutEncoder();
                drivewWithFeedback_FBM(0.5, 0, 0.25);  // Adjust to go backward
                break;
            case 3:  // Target zone C

                stopResetEncoder();  // Use encoders

                setTargetPosition(DIAG_RIGHT_FRONT,1550);  //Strafe at an angle.
                runToPositionDiag_LF_RB();

                runWithoutEncoder();
                drivewWithFeedback_FBM(-0.5, 0, 0.25);  // Go forward

                stopResetEncoder();
                setTargetPosition(DRIVE_FORWARD,2000);  // Just to test IMU - Drive forward.
                runToPosition(0.8);

                setTargetPosition(WOBBLE_ARM_DOWN, 3100);  //Move wobble arm down to place thw wobble goal.
                runToPositionForWobbleArm(0.85);

                hardwarePushBot.wobbleGoalFinger.setPosition(0);  //Release the finger.
                hardwarePushBot.wobbleGoalFinger2.setPosition(0);
                sleep(300);

                setTargetPosition(WOBBLE_ARM_UP, 3100);  //Move wobble arm down to place thw wobble goal.
                runToPositionForWobbleArmBack(0.9);

                runWithoutEncoder();
                drivewWithFeedback_FBM(0.5, 0, 0.25);  // Go backward

                stopResetEncoder();
                setTargetPosition(DRIVE_BACK,1000);
                runToPosition(0.8);

                startShootingWheelUsingEncoder();  //Start shooting wheel using encoder.

                runWithoutEncoder();
                drivewWithFeedback_FBM(0, -0.5, 0.9); //Strafe Left.

                stopResetEncoder();

                //Shoot the rings one by one.
               for (int i=0; i < 3 && opModeIsActive(); i++) {
                    hardwarePushBot.shootingTrigger.setPosition(1);
                    sleep(550);
                    hardwarePushBot.shootingTrigger.setPosition(0);
                    sleep(500);
                }
                hardwarePushBot.shootingWheel.setPower(0);

                setTargetPosition(STRAFE_LEFT, 600);  //Move wobble arm down to place thw wobble goal.
                runToPosition(0.8);

                setTargetPosition(TURN_LEFT,1350);  // Turn around 180 degrees to get second wobble goal using enocder.
                runToPosition(0.9);

                setTargetPosition(WOBBLE_ARM_DOWN, 3100);  // Move the warm slightly up to avoid wobble arm damage.
                runToPositionForWobbleArm(1.0);

                setTargetPosition(DRIVE_FORWARD, 775);  //Move wobble arm down to place thw wobble goal.
                runToPosition(0.8);

                hardwarePushBot.wobbleGoalFinger.setPosition(1);  //Close the finger.
                hardwarePushBot.wobbleGoalFinger2.setPosition(1);
                sleep(400);

                setTargetPosition(WOBBLE_ARM_UP, 3100);  // Move the warm slightly up to avoid wobble arm damage.
                runToPositionForWobbleArm(1.0);

                setTargetPosition(DRIVE_BACK, 1250);  //Move wobble arm down to place thw wobble goal.
                runToPosition(0.8);

                setTargetPosition(TURN_LEFT,1200);  // Turn around 180 degrees to get second wobble goal using enocder.
                runToPosition(0.9);

                setTargetPosition(DRIVE_FORWARD, 900);  //Move wobble arm down to place thw wobble goal.
                runToPosition(0.8);

                setTargetPosition(WOBBLE_ARM_DOWN, 3100);  //Move wobble arm down to place thw wobble goal.
                runToPositionForWobbleArm(0.85);

                hardwarePushBot.wobbleGoalFinger.setPosition(0);  //Release the finger.
                hardwarePushBot.wobbleGoalFinger2.setPosition(0);
                sleep(300);

                setTargetPosition(DRIVE_BACK, 1200);  //Move wobble arm down to place thw wobble goal.
                runToPosition(0.8);
                break;
        }
    }

    /**
     * Drive/Strafe with feedback.
     * @param drive_power
     * @param strafe_power
     */
    public void drivewWithFeedback_Encoders(double drive_power, double strafe_power){

        heading = hardwarePushBot.getAngle();
        error =heading - 0; // desrired - current heading is the error
        integralError = integralError + error*0.025;
        hardwarePushBot.mecanumDrive(drive_power,strafe_power,-(error*GAIN_PROP+integralError*GAIN_INT)); // the multiplication of 0.015 is a gain to make the turn power small (not close to 1, which is maximum)

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

    public void startShootingWheelUsingEncoder(double shootingVelocity) {
        hardwarePushBot.shootingWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardwarePushBot.shootingWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardwarePushBot.shootingWheel.setVelocity(shootingVelocity);

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
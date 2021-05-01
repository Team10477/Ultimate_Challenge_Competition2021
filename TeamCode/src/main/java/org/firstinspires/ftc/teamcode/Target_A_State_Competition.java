package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Target A_State Competition_Red Right Target Zones")
public class Target_A_State_Competition extends LinearOpMode {

    RingIdentification ringIdentification = new RingIdentification();
    EncoderBot encoderBot = new EncoderBot();

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
    static int DRIVE_REVERSE = 12;
    static double SHOOTING_WHEEL_VELOCITY = -1750;
    int leftFrontTargetPosition = 0;
    int rightFrontTargetPosition = 0;
    int leftBackTargetPosition = 0;
    int rightBackTargetPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardwareMap();
        waitForStart();
        int counter = 1;

        while (opModeIsActive() && counter == 1) {
            targetZone = ringIdentification.identifyRings(telemetry);
            // testDriveFBM();
            goToTargetZones();
          /*  double travelDistance = hardwarePushBot.getDistance();
            telemetry.addData("distance", travelDistance);
            telemetry.addData("encoder count", hardwarePushBot.rightFrontWheel.getCurrentPosition() );
            telemetry.update();*/
            ringIdentification.deactivateTfod();
            counter++;
        }

    }

    public void initHardwareMap() {
        hardwarePushBot.initHardwareMap(hardwarePushBot, hardwareMap);

        ringIdentification.initVuforia(hardwareMap);
        ringIdentification.initTfod(hardwareMap);
        ringIdentification.activateTfod();
        sleep(6500);

        telemetry.addData("Initialization complete", "Done");
        telemetry.update();

    }

    public void goToTargetZones() {
        switch (targetZone) {
            case 1: //Target zone A
                goToTargetA();
                break;
            case 2: // Target zone B
                goToTargetB();
                break;
            case 3:  // Target zone C
                goToTargetC();
                break;
        }
    }

    private void goToTargetA() {

        encoderBot.stopResetEncoder(hardwarePushBot);
        encoderBot.runWithoutEncoder(hardwarePushBot);
        drivewWithFeedback_FBM_encoderCount(-0.8,0, DRIVE_FORWARD, -1575);

        //Go to the right
        encoderBot.stopResetEncoder(hardwarePushBot);
        encoderBot.runWithoutEncoder(hardwarePushBot);
        drivewWithFeedback_FBM_distanceSensor(0, 0.4, 1, 8);

        encoderBot.stopResetEncoder(hardwarePushBot);
        //Wobble Goal Arm Down
        encoderBot.setTargetPosition(hardwarePushBot, WOBBLE_ARM_DOWN, 3100);  //Move wobble arm down to place thw wobble goal.
        encoderBot.runToPositionForWobbleArm(hardwarePushBot, 0.75, this);

        //Open Claw
        hardwarePushBot.wobbleGoalFinger.setPosition(0);
        hardwarePushBot.wobbleGoalFinger2.setPosition(0);

        sleep(250);

        //Drive Back
        encoderBot.stopResetEncoder(hardwarePushBot);
        encoderBot.runWithoutEncoder(hardwarePushBot);
        drivewWithFeedback_FBM_encoderCount(0.8,0, DRIVE_BACK, 200);

        encoderBot.stopResetEncoder(hardwarePushBot);

        //Turn Left
        encoderBot.setTargetPosition(hardwarePushBot, TURN_LEFT,1115);
        encoderBot.runToPosition(hardwarePushBot,this,0.8, telemetry);

        //Drive Forward
        encoderBot.stopResetEncoder(hardwarePushBot);
        encoderBot.setTargetPosition(hardwarePushBot, DRIVE_FORWARD,985);
        encoderBot.runToPosition(hardwarePushBot,this,0.6, telemetry);

        //Close Claw
        hardwarePushBot.wobbleGoalFinger.setPosition(1);
        hardwarePushBot.wobbleGoalFinger2.setPosition(1);
        sleep(500);

        //Wobble Goal Arm Up slightly
        encoderBot.setTargetPosition(hardwarePushBot, WOBBLE_ARM_UP, 2000);  //Move the wobble arm up.
        encoderBot.runToPositionForWobbleArmBack(hardwarePushBot, 0.75, this);

        //Drive Back
        encoderBot.stopResetEncoder(hardwarePushBot);
        encoderBot.setTargetPosition(hardwarePushBot, DRIVE_BACK,675);
        encoderBot.runToPosition(hardwarePushBot,this,0.6, telemetry);

        encoderBot.stopResetEncoder(hardwarePushBot);

        //Turn Right
        encoderBot.setTargetPosition(hardwarePushBot, TURN_RIGHT,1200);
        encoderBot.runToPosition(hardwarePushBot,this,0.8, telemetry);

        //Drive Forward\
        encoderBot.stopResetEncoder(hardwarePushBot);
        encoderBot.setTargetPosition(hardwarePushBot, DRIVE_FORWARD,300);
        encoderBot.runToPosition(hardwarePushBot,this,0.6, telemetry);

        //Wobble Goal Arm Up slightly
        encoderBot.setTargetPosition(hardwarePushBot, WOBBLE_ARM_DOWN, 2000);  //Move wobble arm down to place thw wobble goal.
        encoderBot.runToPositionForWobbleArm(hardwarePushBot, 0.75, this);

        hardwarePushBot.wobbleGoalFinger.setPosition(0);
        hardwarePushBot.wobbleGoalFinger2.setPosition(0);
        sleep(10000);

        //Drive Back
        encoderBot.setTargetPosition_FBM(hardwarePushBot, DRIVE_BACK,250);
        encoderBot.runToPosition_FBM(hardwarePushBot,DRIVE_BACK,this,0.8);

        //Wobble Goal Arm Up slightly
        encoderBot.setTargetPosition(hardwarePushBot, WOBBLE_ARM_UP, 3100);  //Move the wobble arm up.
        encoderBot.runToPositionForWobbleArmBack(hardwarePushBot, 0.75, this);

        SHOOTING_WHEEL_VELOCITY = -1700;
        encoderBot.startShootingWheelUsingEncoder(hardwarePushBot,SHOOTING_WHEEL_VELOCITY);

        //Strafe Left

        encoderBot.setTargetPosition_FBM(hardwarePushBot, STRAFE_LEFT,610);
        encoderBot.runToPosition_FBM(hardwarePushBot,STRAFE_LEFT,this,0.8);

        //drivewWithFeedback_FBM(0.6, 0, 0.25);

        hardwarePushBot.shootingTrigger.setPosition(1);
        sleep(550);
        hardwarePushBot.shootingTrigger.setPosition(0);
        sleep(500);

        encoderBot.setTargetPosition(hardwarePushBot, TURN_LEFT,75);
        encoderBot.runToPosition(hardwarePushBot,this,0.6, telemetry);

        encoderBot.setTargetPosition(hardwarePushBot, DRIVE_FORWARD,100);
        encoderBot.runToPosition(hardwarePushBot,this,0.6, telemetry);

        hardwarePushBot.shootingTrigger.setPosition(1);
        sleep(550);
        hardwarePushBot.shootingTrigger.setPosition(0);
        sleep(500);

        encoderBot.setTargetPosition(hardwarePushBot, DRIVE_BACK,100);
        encoderBot.runToPosition(hardwarePushBot,this,0.6, telemetry);

        encoderBot.setTargetPosition(hardwarePushBot, TURN_LEFT,45);
        encoderBot.runToPosition(hardwarePushBot,this,0.6, telemetry);

        encoderBot.setTargetPosition(hardwarePushBot, DRIVE_FORWARD,100);
        encoderBot.runToPosition(hardwarePushBot,this,0.6, telemetry);

        hardwarePushBot.shootingTrigger.setPosition(1);
        sleep(550);
        hardwarePushBot.shootingTrigger.setPosition(0);
        sleep(500);

        encoderBot.setTargetPosition_FBM(hardwarePushBot, DRIVE_FORWARD,100);
        encoderBot.runToPosition_FBM(hardwarePushBot,DRIVE_FORWARD,this,0.6);
    }

    private void goToTargetB() {
        encoderBot.stopResetEncoder(hardwarePushBot);
        encoderBot.runWithoutEncoder(hardwarePushBot);
        drivewWithFeedback_angle_encoderCount(-0.8, 0, DIAG_RIGHT_FRONT, -1600 );

        encoderBot.stopResetEncoder(hardwarePushBot);
        encoderBot.runWithoutEncoder(hardwarePushBot);
        drivewWithFeedback_FBM_encoderCount(-0.8,0, DRIVE_FORWARD, -640);

        encoderBot.stopResetEncoder(hardwarePushBot);
        encoderBot.runWithoutEncoder(hardwarePushBot);
        drivewWithFeedback_angle_encoderCount(-0.8, 0, DIAG_LEFT_FRONT, -1650);

        encoderBot.stopResetEncoder(hardwarePushBot);
        encoderBot.setTargetPosition(hardwarePushBot, WOBBLE_ARM_DOWN, 3100);  //Move wobble arm down to place thw wobble goal.
        encoderBot.runToPositionForWobbleArm(hardwarePushBot, 0.75, this);

        hardwarePushBot.wobbleGoalFinger.setPosition(0);  //Release the finger.
        hardwarePushBot.wobbleGoalFinger2.setPosition(0);
        sleep(200);

        encoderBot.stopResetEncoder(hardwarePushBot);
        encoderBot.runWithoutEncoder(hardwarePushBot);
        drivewWithFeedback_angle_encoderCount(0.8, 0, DIAG_RIGHT_BACK, -1600 );

        //  encoderBot.startShootingWheelUsingEncoder(hardwarePushBot, SHOOTING_WHEEL_VELOCITY);
        //  sleep(200);

        encoderBot.setTargetPosition(hardwarePushBot, WOBBLE_ARM_UP, 3100);  //Move the wobble arm up.
        encoderBot.runToPositionForWobbleArmBack(hardwarePushBot, 0.75, this);

        encoderBot.stopResetEncoder(hardwarePushBot);
        encoderBot.runWithoutEncoder(hardwarePushBot);
        drivewWithFeedback_FBM_encoderCount(0.8,0, DRIVE_BACK, 420);

        drivewWithFeedback_FBM_distanceSensor(0, 0.4, 0.55, 21); //Strafe right to get ready for shooting

        //Shoot the rings one by one.
        for (int i = 0; i < 3 && opModeIsActive(); i++) {
            hardwarePushBot.shootingTrigger.setPosition(1);
            sleep(550);
            hardwarePushBot.shootingTrigger.setPosition(0);
            sleep(500);
        }
        hardwarePushBot.shootingWheel.setPower(0); //turn off the shooting wheel.

        encoderBot.stopResetEncoder(hardwarePushBot);
        encoderBot.runWithoutEncoder(hardwarePushBot);
        drivewWithFeedback_encoderCount_distanceSensor(0.0,-0.6, STRAFE_LEFT, -1400, 35);

        encoderBot.stopResetEncoder(hardwarePushBot);
        encoderBot.setTargetPosition(hardwarePushBot, TURN_LEFT, 1350);
        encoderBot.runToPosition(hardwarePushBot,this, 0.9, telemetry);

        encoderBot.setTargetPosition(hardwarePushBot, WOBBLE_ARM_DOWN, 3100);  //Move wobble arm down to place thw wobble goal.
        encoderBot.runToPositionForWobbleArm(hardwarePushBot, 0.75, this);

        encoderBot.stopResetEncoder(hardwarePushBot);
        encoderBot.setTargetPosition(hardwarePushBot, DRIVE_FORWARD,750);
        encoderBot.runToPosition(hardwarePushBot, this, 0.8, telemetry);

        hardwarePushBot.wobbleGoalFinger.setPosition(1);  //Release the finger.
        hardwarePushBot.wobbleGoalFinger2.setPosition(1);
        sleep(200);

        encoderBot.setTargetPosition(hardwarePushBot, WOBBLE_ARM_UP, 3100);  //Move the wobble arm up.
        encoderBot.runToPositionForWobbleArmBack(hardwarePushBot, 0.75, this);

        encoderBot.setTargetPosition(hardwarePushBot, DRIVE_BACK,800);
        encoderBot.runToPosition(hardwarePushBot, this, 0.8, telemetry);
        //close the finger

        //lift arm

        //go back

        //turn 180

        //drop
        /*
        encoderBot.stopResetEncoder(hardwarePushBot);
        encoderBot.setTargetPosition(hardwarePushBot, TURN_LEFT, 1350);
        encoderBot.runToPosition(hardwarePushBot,this, 0.9, telemetry);

        encoderBot.setTargetPosition(hardwarePushBot, WOBBLE_ARM_DOWN, 3100);  //Move wobble arm down to place thw wobble goal.
        encoderBot.runToPositionForWobbleArm(hardwarePushBot, 0.75, this);

        encoderBot.stopResetEncoder(hardwarePushBot);
        encoderBot.runWithoutEncoder(hardwarePushBot);
        drivewWithFeedback_FBM_encoderCount(-0.8,0, DRIVE_FORWARD, -650);*/

    }

    private void goToTargetB1() {
        encoderBot.stopResetEncoder(hardwarePushBot);
        encoderBot.runWithoutEncoder(hardwarePushBot);
        drivewWithFeedback_angle_encoderCount(-0.8, 0, DIAG_RIGHT_FRONT, -1600 );

        encoderBot.stopResetEncoder(hardwarePushBot);
        encoderBot.runWithoutEncoder(hardwarePushBot);
        drivewWithFeedback_FBM_encoderCount(-0.8,0, DRIVE_FORWARD, -640);

        encoderBot.stopResetEncoder(hardwarePushBot);
        encoderBot.runWithoutEncoder(hardwarePushBot);
        drivewWithFeedback_angle_encoderCount(-0.8, 0, DIAG_LEFT_FRONT, -1650);

        encoderBot.stopResetEncoder(hardwarePushBot);
        encoderBot.setTargetPosition(hardwarePushBot, WOBBLE_ARM_DOWN, 3100);  //Move wobble arm down to place thw wobble goal.
        encoderBot.runToPositionForWobbleArm(hardwarePushBot, 0.75, this);

        hardwarePushBot.wobbleGoalFinger.setPosition(0);  //Release the finger.
        hardwarePushBot.wobbleGoalFinger2.setPosition(0);
        sleep(200);

        encoderBot.startShootingWheelUsingEncoder(hardwarePushBot, SHOOTING_WHEEL_VELOCITY);
        sleep(200);

        encoderBot.setTargetPosition(hardwarePushBot, WOBBLE_ARM_UP, 3100);  //Move the wobble arm up.
        encoderBot.runToPositionForWobbleArmBack(hardwarePushBot, 0.75, this);

        encoderBot.stopResetEncoder(hardwarePushBot);
        encoderBot.runWithoutEncoder(hardwarePushBot);
        drivewWithFeedback_FBM_encoderCount(0.8,0, DRIVE_BACK, 420);

        drivewWithFeedback_FBM_distanceSensor(0, 0.4, 0.55, 21); //Strafe right to get ready for shooting

        //Shoot the rings one by one.
        for (int i = 0; i < 3 && opModeIsActive(); i++) {
            hardwarePushBot.shootingTrigger.setPosition(1);
            sleep(550);
            hardwarePushBot.shootingTrigger.setPosition(0);
            sleep(500);
        }
        hardwarePushBot.shootingWheel.setPower(0); //turn off the shooting wheel.

        encoderBot.stopResetEncoder(hardwarePushBot);
        encoderBot.runWithoutEncoder(hardwarePushBot);
        drivewWithFeedback_encoderCount_distanceSensor(0.0,-0.6, STRAFE_LEFT, -1400, 35);

        encoderBot.stopResetEncoder(hardwarePushBot);
        encoderBot.setTargetPosition(hardwarePushBot, TURN_LEFT, 1350);
        encoderBot.runToPosition(hardwarePushBot,this, 0.9, telemetry);

        encoderBot.setTargetPosition(hardwarePushBot, WOBBLE_ARM_DOWN, 3100);  //Move wobble arm down to place thw wobble goal.
        encoderBot.runToPositionForWobbleArm(hardwarePushBot, 0.75, this);

        encoderBot.stopResetEncoder(hardwarePushBot);
        encoderBot.setTargetPosition(hardwarePushBot, DRIVE_FORWARD,750);
        encoderBot.runToPosition(hardwarePushBot, this, 0.8, telemetry);

        hardwarePushBot.wobbleGoalFinger.setPosition(1);  //Release the finger.
        hardwarePushBot.wobbleGoalFinger2.setPosition(1);
        sleep(200);

        encoderBot.setTargetPosition(hardwarePushBot, WOBBLE_ARM_UP, 3100);  //Move the wobble arm up.
        encoderBot.runToPositionForWobbleArmBack(hardwarePushBot, 0.75, this);

        encoderBot.setTargetPosition(hardwarePushBot, DRIVE_BACK,800);
        encoderBot.runToPosition(hardwarePushBot, this, 0.8, telemetry);
        //close the finger

        //lift arm

        //go back

        //turn 180

        //drop
        /*
        encoderBot.stopResetEncoder(hardwarePushBot);
        encoderBot.setTargetPosition(hardwarePushBot, TURN_LEFT, 1350);
        encoderBot.runToPosition(hardwarePushBot,this, 0.9, telemetry);

        encoderBot.setTargetPosition(hardwarePushBot, WOBBLE_ARM_DOWN, 3100);  //Move wobble arm down to place thw wobble goal.
        encoderBot.runToPositionForWobbleArm(hardwarePushBot, 0.75, this);

        encoderBot.stopResetEncoder(hardwarePushBot);
        encoderBot.runWithoutEncoder(hardwarePushBot);
        drivewWithFeedback_FBM_encoderCount(-0.8,0, DRIVE_FORWARD, -650);*/

    }

    private void goToTargetB_Old() {
        encoderBot.stopResetEncoder(hardwarePushBot);
        encoderBot.setTargetPosition(hardwarePushBot, DIAG_RIGHT_FRONT, 1600);
        encoderBot.runToPositionDiag_LF_RB(hardwarePushBot, this);

        encoderBot.setTargetPosition_FBM(hardwarePushBot, DRIVE_FORWARD, 700);
        encoderBot.runToPosition_FBM(hardwarePushBot, DRIVE_FORWARD, this, 0.85);

        encoderBot.setTargetPosition(hardwarePushBot, DIAG_LEFT_FRONT, 1500);
        encoderBot.runToPositionDiag_RF_LB(hardwarePushBot, this);

        encoderBot.setTargetPosition(hardwarePushBot, WOBBLE_ARM_DOWN, 3100);  //Move wobble arm down to place thw wobble goal.
        encoderBot.runToPositionForWobbleArm(hardwarePushBot, 0.75, this);

        hardwarePushBot.wobbleGoalFinger.setPosition(0);  //Release the finger.
        hardwarePushBot.wobbleGoalFinger2.setPosition(0);
        sleep(200);

        encoderBot.startShootingWheelUsingEncoder(hardwarePushBot, SHOOTING_WHEEL_VELOCITY);
        sleep(200);

        encoderBot.setTargetPosition(hardwarePushBot, WOBBLE_ARM_UP, 3100);  //Move the wobble arm up.
        encoderBot.runToPositionForWobbleArmBack(hardwarePushBot, 0.75, this);


        encoderBot.runWithoutEncoder(hardwarePushBot);
        drivewWithFeedback_FBM(0.4, 0, .85); //Drive back

        drivewWithFeedback_FBM_distanceSensor(0, 0.4, 0.55, 18); //Strafe right to get ready for shooting

        encoderBot.stopResetEncoder(hardwarePushBot);

        //Shoot the rings one by one.
        for (int i = 0; i < 3 && opModeIsActive(); i++) {
            hardwarePushBot.shootingTrigger.setPosition(1);
            sleep(550);
            hardwarePushBot.shootingTrigger.setPosition(0);
            sleep(500);
        }
        hardwarePushBot.shootingWheel.setPower(0); //turn off the shooting wheel

        encoderBot.setTargetPosition(hardwarePushBot, STRAFE_LEFT, 675);  // Drive forward.
        encoderBot.runToPosition_withdistanceSensor(hardwarePushBot, this, 0.8, 38, telemetry);

        encoderBot.setTargetPosition(hardwarePushBot, TURN_LEFT, 1350);
        encoderBot.runToPosition(hardwarePushBot,this, 0.9, telemetry);

        encoderBot.setTargetPosition(hardwarePushBot, WOBBLE_ARM_DOWN, 3100);  //Move wobble arm down to place thw wobble goal.
        encoderBot.runToPositionForWobbleArm(hardwarePushBot, 0.75, this);

        encoderBot.setTargetPosition(hardwarePushBot, DRIVE_FORWARD, 600);
        encoderBot.runToPosition(hardwarePushBot,this, 0.9, telemetry);
    }

    private void goToTargetC() {
        encoderBot.stopResetEncoder(hardwarePushBot);  // Use encoders
        encoderBot.setTargetPosition(hardwarePushBot, DIAG_RIGHT_FRONT, 1550);  //Strafe at an angle.
        encoderBot.runToPositionDiag_LF_RB(hardwarePushBot, this);

        encoderBot.runWithoutEncoder(hardwarePushBot);
        drivewWithFeedback_FBM(-0.5, 0, 0.25);  // Go forward

        encoderBot.stopResetEncoder(hardwarePushBot);
        encoderBot.setTargetPosition(hardwarePushBot, DRIVE_FORWARD, 2000);  // Just to test IMU - Drive forward.
        encoderBot.runToPosition(hardwarePushBot, this, 0.8, telemetry);

        encoderBot.setTargetPosition(hardwarePushBot, WOBBLE_ARM_DOWN, 3100);  //Move wobble arm down to place thw wobble goal.
        encoderBot.runToPositionForWobbleArm(hardwarePushBot, 0.85, this);

        hardwarePushBot.wobbleGoalFinger.setPosition(0);  //Release the finger.
        hardwarePushBot.wobbleGoalFinger2.setPosition(0);
        sleep(300);

        encoderBot.setTargetPosition(hardwarePushBot, WOBBLE_ARM_UP, 3100);  //Move wobble arm down to place thw wobble goal.
        encoderBot.runToPositionForWobbleArmBack(hardwarePushBot, 0.9, this);

        encoderBot.runWithoutEncoder(hardwarePushBot);
        drivewWithFeedback_FBM(0.5, 0, 0.25);  // Go backward

        encoderBot.stopResetEncoder(hardwarePushBot);
        encoderBot.setTargetPosition(hardwarePushBot, DRIVE_BACK, 1000);
        encoderBot.runToPosition(hardwarePushBot, this, 0.8, telemetry);

        encoderBot.startShootingWheelUsingEncoder(hardwarePushBot, SHOOTING_WHEEL_VELOCITY);  //Start shooting wheel using encoder.

        encoderBot.runWithoutEncoder(hardwarePushBot);
        //    drivewWithFeedback_FBM(0, -0.5, 0.9); //Strafe Left.
        drivewWithFeedback_FBM_distanceSensor(0, -0.5, 0.9, 23);

        encoderBot.stopResetEncoder(hardwarePushBot);

        //Shoot the rings one by one.
        for (int i = 0; i < 3 && opModeIsActive(); i++) {
            hardwarePushBot.shootingTrigger.setPosition(1);
            sleep(550);
            hardwarePushBot.shootingTrigger.setPosition(0);
            sleep(500);
        }
        hardwarePushBot.shootingWheel.setPower(0);

        encoderBot.setTargetPosition(hardwarePushBot, STRAFE_LEFT, 600);  //Move wobble arm down to place thw wobble goal.
        encoderBot.runToPosition(hardwarePushBot, this, 0.8, telemetry);

        encoderBot.setTargetPosition(hardwarePushBot, TURN_LEFT, 1350);  // Turn around 180 degrees to get second wobble goal using enocder.
        encoderBot.runToPosition(hardwarePushBot, this, 0.9, telemetry);

        encoderBot.setTargetPosition(hardwarePushBot, WOBBLE_ARM_DOWN, 3100);  // Move the warm slightly up to avoid wobble arm damage.
        encoderBot.runToPositionForWobbleArm(hardwarePushBot, 0.9, this);

        encoderBot.setTargetPosition(hardwarePushBot, DRIVE_FORWARD, 775);  //Move wobble arm down to place thw wobble goal.
        encoderBot.runToPosition(hardwarePushBot, this, 0.8, telemetry);

        hardwarePushBot.wobbleGoalFinger.setPosition(1);  //Close the finger.
        hardwarePushBot.wobbleGoalFinger2.setPosition(1);
        sleep(400);

        encoderBot.setTargetPosition(hardwarePushBot, WOBBLE_ARM_UP, 3100);  // Move the warm slightly up to avoid wobble arm damage.
        encoderBot.runToPositionForWobbleArmBack(hardwarePushBot, 1.0, this);

        encoderBot.setTargetPosition(hardwarePushBot, DRIVE_BACK, 1250);  //Move wobble arm down to place thw wobble goal.
        encoderBot.runToPosition(hardwarePushBot, this, 0.8, telemetry);

        encoderBot.setTargetPosition(hardwarePushBot, TURN_LEFT, 1200);  // Turn around 180 degrees to get second wobble goal using enocder.
        encoderBot.runToPosition(hardwarePushBot, this, 0.9, telemetry);

        encoderBot.setTargetPosition(hardwarePushBot, DRIVE_FORWARD, 900);  //Move wobble arm down to place thw wobble goal.
        encoderBot.runToPosition(hardwarePushBot, this, 0.8, telemetry);

        encoderBot.setTargetPosition(hardwarePushBot, WOBBLE_ARM_DOWN, 3100);  //Move wobble arm down to place thw wobble goal.
        encoderBot.runToPositionForWobbleArm(hardwarePushBot, 0.85, this);

        hardwarePushBot.wobbleGoalFinger.setPosition(0);  //Release the finger.
        hardwarePushBot.wobbleGoalFinger2.setPosition(0);
        sleep(300);

        encoderBot.setTargetPosition(hardwarePushBot, DRIVE_BACK, 1200);  //Move wobble arm down to place thw wobble goal.
        encoderBot.runToPosition(hardwarePushBot, this, 0.8, telemetry);
    }

    private void testDriveFBM() {
        encoderBot.stopResetEncoder(hardwarePushBot);
        encoderBot.runWithoutEncoder(hardwarePushBot);
        telemetry.addData("current position before", hardwarePushBot.rightFrontWheel.getCurrentPosition());
        telemetry.update();
        // encoderBot.setTargetPosition_FBM(hardwarePushBot, DRIVE_FORWARD, 3000);
        // encoderBot.runToPosition_FBM(hardwarePushBot, DRIVE_FORWARD, this, 0.85);
        drivewWithFeedback_FBM_encoderCount(-0.8, 0,  DRIVE_FORWARD, -2500);
        telemetry.addData("current position after", hardwarePushBot.rightFrontWheel.getCurrentPosition());
        telemetry.update();
        sleep(5000);
    }

    /**
     * Drive/Strafe with feedback.
     *
     * @param drive_power
     * @param strafe_power
     * @param timeOut
     */
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


    /**
     * Drive/Strafe with feedback.
     *
     * @param drive_power
     * @param strafe_power
     */
    public void drivewWithFeedback_FBM_encoderCount(double drive_power, double strafe_power, int action, double encoderCount) {
        hardwarePushBot.mecanumDrive(drive_power, strafe_power, 0.0); // pass the parameters to a mecanumDrive method

        elapsedTime.reset();
        integralError = 0;
        encoderBot.setBrake(hardwarePushBot);

        if (action == 0) { //Drive back
            while (opModeIsActive() && hardwarePushBot.rightFrontWheel.getCurrentPosition() <= encoderCount) {
                drive_with_correction(drive_power, strafe_power);

            }
        } else  if (action == 1) { //Drive forward
            while (opModeIsActive() && hardwarePushBot.rightFrontWheel.getCurrentPosition() >= encoderCount) {
                drive_with_correction(drive_power, strafe_power);

            }
        } else  if (action == 2) { //Strafe Right
            while (opModeIsActive() && hardwarePushBot.rightBackWheel.getCurrentPosition() >= encoderCount) {
                drive_with_correction(drive_power, strafe_power);

            }
        }  else  if (action == 3) { //Strafe Left
            while (opModeIsActive() && hardwarePushBot.rightFrontWheel.getCurrentPosition() >= encoderCount) {
                drive_with_correction(drive_power, strafe_power);

            }
        }  else  if (action == TURN_LEFT) { //Turn Left
            while (opModeIsActive() && hardwarePushBot.rightFrontWheel.getCurrentPosition() <= encoderCount) {
                drive_with_correction(drive_power, strafe_power);

            }
        }  else  if (action == DRIVE_REVERSE) { //Drive opposite
            while (opModeIsActive() && hardwarePushBot.rightFrontWheel.getCurrentPosition() > encoderCount) {
                drive_with_correction_driveback(drive_power, strafe_power);

            }
        }
        hardwarePushBot.mecanumDrive(0, 0.0, 0.0);

    }

    /**
     * Drive/Strafe with feedback.
     *
     * @param drive_power
     * @param strafe_power
     */
    public void drivewWithFeedback_encoderCount_distanceSensor(double drive_power, double strafe_power, int action, double encoderCount, int distance) {
        hardwarePushBot.mecanumDrive(drive_power, strafe_power, 0.0); // pass the parameters to a mecanumDrive method

        elapsedTime.reset();
        integralError = 0;
        encoderBot.setBrake(hardwarePushBot);
        double travelDistance = hardwarePushBot.getDistance();

        if (action == 0) { //Drive back
            while (opModeIsActive() && hardwarePushBot.rightFrontWheel.getCurrentPosition() <= encoderCount &&  hardwarePushBot.getDistance() >= distance) {
                drive_with_correction(drive_power, strafe_power);

            }
        } else  if (action == 1) { //Drive forward
            while (opModeIsActive() && hardwarePushBot.rightFrontWheel.getCurrentPosition() >= encoderCount && hardwarePushBot.getDistance() <= distance) {
                drive_with_correction(drive_power, strafe_power);

            }
        } else  if (action == 2) { //Strafe Right
            while (opModeIsActive() && hardwarePushBot.rightBackWheel.getCurrentPosition() >= encoderCount &&  hardwarePushBot.getDistance() >= distance) {
                drive_with_correction(drive_power, strafe_power);

            }
        }  else  if (action == 3) { //Strafe Left
            while (opModeIsActive() && hardwarePushBot.rightFrontWheel.getCurrentPosition() >= encoderCount &&  travelDistance <= distance) {
                drive_with_correction(drive_power, strafe_power);
                travelDistance = hardwarePushBot.getDistance();

            }
        }
        hardwarePushBot.mecanumDrive(0, 0.0, 0.0);

    }


    /**
     * Drive/Strafe at an angle with feedback.
     *
     * @param drive_power
     * @param strafe_power
     */
    public void drivewWithFeedback_angle_encoderCountbk(double drive_power, double strafe_power,  int action, int position) {
        hardwarePushBot.mecanumDrive(drive_power, strafe_power, 0.0); // pass the parameters to a mecanumDrive method

        elapsedTime.reset();
        integralError = 0;
        encoderBot.setBrake(hardwarePushBot);

        if (action == DIAG_RIGHT_FRONT) {
            leftFrontTargetPosition = hardwarePushBot.leftFrontWheel.getCurrentPosition() - position;
            while (opModeIsActive() && hardwarePushBot.leftFrontWheel.getCurrentPosition() > leftFrontTargetPosition) {
                drive_with_angle_correction(drive_power, strafe_power, 1, 4);
            }

        } else if (action == DIAG_LEFT_FRONT) {
            rightFrontTargetPosition = hardwarePushBot.rightFrontWheel.getCurrentPosition() - position;
            while (opModeIsActive() && hardwarePushBot.rightFrontWheel.getCurrentPosition() > rightFrontTargetPosition) {
                drive_with_angle_correction(drive_power, strafe_power, 2, 3);
            }
        } else if (action == DIAG_LEFT_BACK) {
            leftFrontTargetPosition = hardwarePushBot.leftFrontWheel.getCurrentPosition() + position;
            while (opModeIsActive() && hardwarePushBot.leftFrontWheel.getCurrentPosition() < leftFrontTargetPosition) {
                drive_with_angle_correction(drive_power, strafe_power, 1, 4);
            }
        }  else if (action == DIAG_RIGHT_BACK) {
            rightFrontTargetPosition = hardwarePushBot.rightFrontWheel.getCurrentPosition() + position;
            while (opModeIsActive() && hardwarePushBot.rightFrontWheel.getCurrentPosition() < rightFrontTargetPosition) {
                drive_with_angle_correction(drive_power, strafe_power, 2, 3);
            }
        }
        hardwarePushBot.mecanumDrive(0, 0.0, 0.0);
    }

    /**
     * Drive/Strafe at an angle with feedback.
     *
     * @param drive_power
     * @param strafe_power
     */
    public void drivewWithFeedback_angle_encoderCount(double drive_power, double strafe_power,  int action, double encoderCount) {
        hardwarePushBot.mecanumDrive(drive_power, strafe_power, 0.0); // pass the parameters to a mecanumDrive method

        elapsedTime.reset();
        integralError = 0;
        encoderBot.setBrake(hardwarePushBot);

        if (action == DIAG_RIGHT_FRONT) {
            while (opModeIsActive() && hardwarePushBot.leftFrontWheel.getCurrentPosition() >= encoderCount) {
                drive_with_angle_correction(drive_power, strafe_power, 1, 4);
            }

        } else if (action == DIAG_LEFT_FRONT) {
            while (opModeIsActive() && hardwarePushBot.rightFrontWheel.getCurrentPosition() >= encoderCount) {
                drive_with_angle_correction(drive_power, strafe_power, 2, 3);
            }
        } else if (action == 10) {
            while (opModeIsActive() && hardwarePushBot.leftFrontWheel.getCurrentPosition() <= encoderCount) {
                drive_with_angle_correction(drive_power, strafe_power, 1, 4);
            }
        }  else if (action == 11) {
            while (opModeIsActive() && hardwarePushBot.rightFrontWheel.getCurrentPosition() <= encoderCount) {
                drive_with_angle_correction(drive_power, strafe_power, 2, 3);
            }
        }
        hardwarePushBot.mecanumDrive(0, 0.0, 0.0);
    }

    private void drive_with_correction_driveback(double drive_power, double strafe_power) {
        heading = hardwarePushBot.getAngle();
        error = 90 - heading ; // desrired - current heading is the error
        integralError = integralError + error * 0.025;

        hardwarePushBot.mecanumDrive(drive_power, strafe_power, -(error * GAIN_PROP + integralError * GAIN_INT)); // the multiplication of 0.015 is a gain to make the turn power small (not close to 1, which is maximum)
        telemetry.addData("current position", hardwarePushBot.rightFrontWheel.getCurrentPosition());
        telemetry.update();
    }

    private void drive_with_correction(double drive_power, double strafe_power) {
        heading = hardwarePushBot.getAngle();
        error = heading - 0; // desrired - current heading is the error
        integralError = integralError + error * 0.025;

        hardwarePushBot.mecanumDrive(drive_power, strafe_power, -(error * GAIN_PROP + integralError * GAIN_INT)); // the multiplication of 0.015 is a gain to make the turn power small (not close to 1, which is maximum)
        telemetry.addData("current position", hardwarePushBot.rightFrontWheel.getCurrentPosition());
        telemetry.update();
    }

    private void drive_with_angle_correction(double drive_power, double strafe_power, int wheelPosition1, int wheelPosition2) {
        heading = hardwarePushBot.getAngle();
        error = heading - 0; // desrired - current heading is the error
        integralError = integralError + error * 0.025;

        hardwarePushBot.mecanumDrive_DiagStrafe(drive_power, strafe_power , -(error * GAIN_PROP + integralError * GAIN_INT), wheelPosition1, wheelPosition2);
        telemetry.addData("current position", hardwarePushBot.rightFrontWheel.getCurrentPosition());
        telemetry.update();
    }

    /**
     * Drive/Strafe with feedback.
     *
     * @param drive_power
     * @param strafe_power
     * @param timeOut
     */
    public void drivewWithFeedback_FBM_distanceSensor(double drive_power, double strafe_power, double timeOut, int distance) {
        hardwarePushBot.mecanumDrive(drive_power, strafe_power, 0.0); // pass the parameters to a mecanumDrive method

        elapsedTime.reset();
        integralError = 0;
        double travelDistance = hardwarePushBot.getDistance();
        //  while (opModeIsActive() && elapsedTime.seconds()<timeOut &&
        while (opModeIsActive() &&
                ((travelDistance <= distance && strafe_power < 0) || (strafe_power > 0 && travelDistance >= distance))) {
            telemetry.addData("distance", String.format("%.01f in inches", travelDistance));
            telemetry.update();
            heading = hardwarePushBot.getAngle();
            error = heading - 0; // desrired - current heading is the error
            integralError = integralError + error * 0.025;
            hardwarePushBot.mecanumDrive(drive_power, strafe_power, -(error * GAIN_PROP + integralError * GAIN_INT)); // the multiplication of 0.015 is a gain to make the turn power small (not close to 1, which is maximum)
            travelDistance = hardwarePushBot.getDistance();
        }
        hardwarePushBot.mecanumDrive(0, 0.0, 0.0);
    }


    /**
     * Drive/Strafe with feedback loop and color detection.
     *
     * @param drive_power
     * @param strafe_power
     * @param timeOut
     * @param colorString
     */

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

    /**
     * RED Color detection.
     *
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
        if ((hue < 60 || hue > 320)) {
            found = true;
            sleep(50);
        }
        return found;
    }

    /**
     * White color detection.
     *
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




}

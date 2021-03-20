package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Encoder-Red Right Target Zones")
public class RightRedTargetZones_Encoder extends LinearOpMode {

    RingIdentification ringIdentification = new RingIdentification();

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

    @Override
    public void runOpMode() throws InterruptedException {
        initHardwareMap();
        waitForStart();
        int counter = 1;
        while (opModeIsActive() && counter == 1) {
           targetZone =  ringIdentification.identifyRings(this.telemetry);
           goToTargetZones();
           counter++;
        }
        ringIdentification.deactivateTfod();
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
        ringIdentification.initVuforia(hardwareMap);

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

        while (opModeIsActive() && hardwarePushBot.wobbleGoalArm.isBusy()) {
            telemetry.addData("Current position wobble arm", hardwarePushBot.wobbleGoalArm.getCurrentPosition());
            telemetry.update();
        }

        hardwarePushBot.wobbleGoalArm.setPower(0);

    }

    public void startShootingWheelByVoltage() {
        double battVoltage= getVoltage();
        hardwarePushBot.shootingWheel.setPower(0.748-(battVoltage-12)*(0.748-0.62)/(1.95));

    }

    public void goToTargetZones() {
        switch (targetZone) {
            case 0: //Target zone A

                //1. Strafe Right

                stopResetEncoder();

                setTargetPosition(STRAFE_RIGHT,1550);

                runToPosition(1);

                //2. Go forward.

                setTargetPosition(DRIVE_FORWARD,3550);

                runToPosition(1);

                //3. Wobble Goal arm down

               setTargetPosition(WOBBLE_ARM_DOWN, 3100);

               runToPositionForWobbleArm(1.0);

               hardwarePushBot.wobbleGoalFinger.setPosition(0);

               //4. Wobble Goal Arm up
                setTargetPosition(WOBBLE_ARM_UP, 3100);

                runToPositionForWobbleArm(1.0);

                //5. Turn on shooting wheel.
              //  hardwarePushBot.shootingWheel.setPower(0.70);

                startShootingWheelByVoltage();

                runWithoutEncoder();

                //6. Go back 0.5 using feedback
                drivewWithFeedback_FBM_Colors(0.5, 0, 1.0, "RED");

                //7. Strafe left the forward using feedback
               drivewWithFeedback_FBM(0, -0.35, 1.65);

               drivewWithFeedback_FBM(-0.5, 0, 0.45);
                //stopResetEncoder();
                //setTargetPosition(STRAFE_LEFT,1000);
                //runToPosition(0.75);

               // setTargetPosition(DRIVE_FORWARD,200);
                //runToPosition(0.75);

                //8.Shooting Rings
                startShootingWheelByVoltage();
                sleep(500);
                for (int i=0; i < 3; i++) {
                    hardwarePushBot.shootingTrigger.setPosition(1);
                    sleep(500);
                    hardwarePushBot.shootingTrigger.setPosition(0);
                    sleep(500);
                }
                hardwarePushBot.shootingWheel.setPower(0); //turn off the shooting wheel

                //9. Strafe Left
                stopResetEncoder();
                setTargetPosition(STRAFE_LEFT,1150);

                runToPosition(1.0);

                //10. Turn around 180 degrees

                setTargetPosition(TURN_RIGHT,2700);
                runToPosition(1.0);

                //11. Move forward to the second wobble goal
                setTargetPosition(DRIVE_FORWARD,1200);
                runToPosition(1.0);

                //12. Wobble Goal arm down

                setTargetPosition(WOBBLE_ARM_DOWN, 3100);

                runToPositionForWobbleArm(0.75);

                hardwarePushBot.wobbleGoalFinger.setPosition(0);

                //13. Move forward to pick up the second wobble goal
                setTargetPosition(DRIVE_FORWARD,800);
                runToPosition(1.0);

                //14. close wobble finger
                hardwarePushBot.wobbleGoalFinger.setPosition(1.0);
                hardwarePushBot.wobbleGoalFinger2.setPosition(1.0);

                hardwarePushBot.wobbleGoalArm.setPower(-0.7);
                sleep(1000);
                hardwarePushBot.wobbleGoalArm.setPower(0);

                //15. Turn around 180 deg
                setTargetPosition(TURN_RIGHT,2700);
                runToPosition(1.0);

                //16. move forward
                setTargetPosition(DRIVE_FORWARD,2000);
                runToPosition(1.0);

                //15. Strafe Right
                setTargetPosition(STRAFE_RIGHT,1500);
                runToPosition(1.0);


                break;
            case 1 : // Target zone B

                //1. Go to the right

                stopResetEncoder();

                setTargetPosition(STRAFE_RIGHT,1300);

                runToPosition(1);

                //2. Go forward.

                setTargetPosition(DRIVE_FORWARD,5000);

                runToPosition(1);

                //3. Strafe right

                setTargetPosition(STRAFE_LEFT,1500);

                runToPosition(1);

                //4. Wobble Goal arm down.

                setTargetPosition(WOBBLE_ARM_DOWN, 3100);

                runToPositionForWobbleArm(0.75);

                hardwarePushBot.wobbleGoalFinger.setPosition(0);

                //4. Wobble Goal Arm up
                setTargetPosition(WOBBLE_ARM_UP, 3100);

                runToPositionForWobbleArm(0.75);

                //3. Strafe right

                setTargetPosition(STRAFE_RIGHT,1500);

                runToPosition(1);

                runWithoutEncoder();

                //6. Go back 0.5 using feedback
                drivewWithFeedback_FBM_Colors(0.5, 0, 3.0, "RED");

                //5. Turn on shooting wheel.
              //  hardwarePushBot.shootingWheel.setPower (0.7)    ;
                startShootingWheelByVoltage();

                //7. Strafe left using feedback
                drivewWithFeedback_FBM(0, -0.5, 0.9);

           //    drivewWithFeedback_FBM(-0.5, 0, 0.25);

                //8.Shooting Rings
                for (int i=0; i < 3; i++) {
                    hardwarePushBot.shootingTrigger.setPosition(1);
                    sleep(500);
                    hardwarePushBot.shootingTrigger.setPosition(0);
                    sleep(500);
                }
                break;
            case 4:  // Target zone C

                //1. Go to the right

                stopResetEncoder();

                setTargetPosition(STRAFE_RIGHT,1300);

                runToPosition(1);

                //2. Go forward.

                setTargetPosition(DRIVE_FORWARD,6300);

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
             //   hardwarePushBot.shootingWheel.setPower(0.7);
                startShootingWheelByVoltage();

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

    public void drivewWithFeedback_FBMEncoder_Heading(double drive_power, double strafe_power, double desiredHeading){
        hardwarePushBot.mecanumDrive(drive_power,strafe_power,0.0); // pass the parameters to a mecanumDrive method

        elapsedTime.reset();
        integralError=0;
        while (opModeIsActive()){
            heading = hardwarePushBot.getAngle();
            error =heading - desiredHeading; // desrired - current heading is the error
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
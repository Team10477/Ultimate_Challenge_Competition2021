/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Iterative OpMode V1", group="Tele Op")
//@Disabled
public class Teleop_Basic_Iterative_v1 extends OpMode
{

     // Setup a variable for each drive wheel to save power level for telemetry
    double leftFrontPower;
    double rightFrontPower;
    double leftRearPower;
    double rightRearPower;
    double wobbleArmPower;
    double wobbleFingerPower;
    double wobbleFinger2Power;
    double wobbleArmPosition;
    double wobbleHandPower;
    double hue, saturation, value;
    double lightIntensity;
    double wobbleGoalupdown=0;
    double battVoltage = 0;
    static double SHOOTING_WHEEL_VELOCITY = -1760;
    static double SHOOTING_WHEEL_VELOCITY_POWERSHOT = -1700;
    DistanceSensor rightSensorRange;
    double heading;
    double integralError = 0;
    double error = 0;
    double deltaTurn = 0;
    double GAIN_PROP = 0.015;
    double GAIN_INT = 0.015;


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    //private Servo frontArm = null;

    HardwarePushBot hardwarePushBot = new HardwarePushBot();

    double masterPowerScaleDrive = 1.0;
    double masterPowerScaleTurn = 1.0;
    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    //final double SCALE_FACTOR = 255;
    final double SCALE_FACTOR = 8;
    // get a reference to the RelativeLayout so we can change the background
    // color of the Robot Controller app to match the hue detected by the RGB sensor.
    //   int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
    //   final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
    /*
     * Code to run ONCE when the driver hits INIT
     */

    public HardwarePushBot getHardwarePushBot() {
        return hardwarePushBot;
    }


    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        initHardwareMap();


        hardwarePushBot.wobbleGoalFinger.setPosition(0.0); // closed finger
        hardwarePushBot.wobbleGoalFinger2.setPosition(0.0);
       // hardwarePushBot.wobbleGoalArm.setDirection(DcMotor.Direction.REVERSE);
        //hardwarePushBot.wobbleGoalArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // hardwarePushBot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //hardwarePushBot.shootingWheel.setDirection(DcMotor.Direction.FORWARD);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        // Tell the driver that initialization is complete.

        telemetry.addData("Status", "Initialized");
        battVoltage= getVoltage();


        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override


    public void loop() {
     /*   // Setup a variable for each drive wheel to save power level for telemetry
        double leftFrontPower;
        double rightFrontPower;
        double leftRearPower;
        double rightRearPower;
        double wobbleArmPower;
        double wobbleFingerPower;
        double wobbleFinger2Power;
        double wobbleArmPosition;
        double wobbleHandPower;
        double hue, saturation, value;
        double lightIntensity;
        double wobbleGoalupdown=0;*/

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = gamepad1.left_stick_y;
        double strafe  =  gamepad1.left_stick_x + gamepad1.right_trigger - gamepad1.left_trigger;
        double turn  =  -gamepad1.right_stick_x;

        if(gamepad1.dpad_up || gamepad2.left_trigger > 0){
            hardwarePushBot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wobbleGoalupdown = 0.7;
        }
        else if(gamepad1.dpad_down || gamepad2.right_trigger > 0){
            hardwarePushBot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wobbleGoalupdown = -0.7;
        }
        else {
            wobbleGoalupdown = 0;
        }
        if(gamepad1.x){
            hardwarePushBot.wobbleGoalFinger.setPosition(1.0);
            hardwarePushBot.wobbleGoalFinger2.setPosition(1.0);
        }
        if(gamepad1.b){
            hardwarePushBot.wobbleGoalFinger.setPosition(0.0);
            hardwarePushBot.wobbleGoalFinger2.setPosition(0.0);
        }

        hardwarePushBot.wobbleGoalArm.setPower(wobbleGoalupdown); // Wobble Goal Arm Position Control

       if(gamepad1.right_bumper){
            masterPowerScaleDrive = 0.8; //Range.clip(masterPowerScaleDrive + 0.2, 0.2, 1.0);
        }

       if(gamepad1.left_bumper){
            masterPowerScaleDrive = 0.4; //Range.clip(masterPowerScaleDrive - 0.2, 0.2, 1.0);
        }

        drive = drive*masterPowerScaleDrive;
        strafe = strafe*masterPowerScaleDrive;
        turn = turn*0.6;

        leftFrontPower   = Range.clip(drive+turn-strafe , -1.0, 1.0);
        rightFrontPower  = Range.clip(drive-turn+strafe , -1.0, 1.0);
        leftRearPower    = Range.clip(drive+turn+strafe , -1.0, 1.0);
        rightRearPower   = Range.clip(drive-turn-strafe , -1.0, 1.0);



        if (gamepad2.left_bumper){
            //set the front arm to an initial position
            hardwarePushBot.shootingTrigger.setPosition(0.0);
        }
        if (gamepad2.right_bumper){
            //set the front arm to an initial position
            hardwarePushBot.shootingTrigger.setPosition(1.0);
        }


        /*if (gamepad2.start){
            hardwarePushBot.ringIntake.setPower(1);
        }*/

        if (gamepad2.back){
            hardwarePushBot.ringIntake.setPower(0);
        }

        if(gamepad2.dpad_down) {
            hardwarePushBot.ringIntake.setPower(1);
        }

        if(gamepad2.dpad_up) {
            hardwarePushBot.ringIntake.setPower(0);
        }

        if(gamepad2.dpad_right|| gamepad2.dpad_left) {
            hardwarePushBot.ringIntake.setPower(-1);
        }

        battVoltage= getVoltage();

        if (gamepad2.a){
            hardwarePushBot.shootingWheel.setPower(0.0);
        }
        if (gamepad2.b){
         //   hardwarePushBot.shootingWheel.setPower(0.748-(battVoltage-12)*(0.748-0.62)/(1.95));
            //startShootingWheelUsingEncoder(SHOOTING_WHEEL_VELOCITY);
            hardwarePushBot.shootingWheel.setVelocity(hardwarePushBot.shootingWheel.getVelocity()-5);
        }
        if (gamepad2.y){
           startShootingWheelUsingEncoder(SHOOTING_WHEEL_VELOCITY);
        }
        if (gamepad2.x){
            //   hardwarePushBot.shootingWheel.setPower(0.748-(battVoltage-12)*(0.748-0.62)/(1.95));
            //startShootingWheelUsingEncoder(SHOOTING_WHEEL_VELOCITY);
            hardwarePushBot.shootingWheel.setVelocity(hardwarePushBot.shootingWheel.getVelocity()+5);
        }
        if (gamepad2.start){
            startShootingWheelUsingEncoder(SHOOTING_WHEEL_VELOCITY_POWERSHOT);
        }

/***
 //      Left Front = +Speed + Turn - Strafe      Right Front = +Speed - Turn + Strafe
 //      Left Rear  = +Speed + Turn + Strafe      Right Rear  = +Speed - Turn - Strafe
 ***/
        // Send calculated power to wheels
        hardwarePushBot.leftFrontWheel.setPower(leftFrontPower);
        hardwarePushBot.rightFrontWheel.setPower(rightFrontPower);
        hardwarePushBot.leftBackWheel.setPower(leftRearPower);
        hardwarePushBot.rightBackWheel.setPower(rightRearPower);

        // Servo Control

        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)

        Color.RGBToHSV((int) (hardwarePushBot.rightColorSensor.red() * SCALE_FACTOR),
                (int) (hardwarePushBot.rightColorSensor.green() * SCALE_FACTOR),
                (int) (hardwarePushBot.rightColorSensor.blue() * SCALE_FACTOR),
                hsvValues);
        // CheckForRed is a hsv value check

        hue = hsvValues[0];
        saturation = hsvValues[1];
        value = hsvValues[2];

        lightIntensity = hardwarePushBot.rightColorSensor.alpha();

        telemetry.addData("Hue", "Hue (%.2f), Sat (%.2f), Val (%.2f)",hue, saturation, value);
        telemetry.addData("LightIntensity",lightIntensity);
        telemetry.addData("Arm Pos", hardwarePushBot.wobbleGoalArm.getCurrentPosition());
        //telemetry.addData("Wobble Power", 0.748-(battVoltage-12)*(0.748-0.62)/(1.95));
        telemetry.addData("Shooting Speed:", hardwarePushBot.shootingWheel.getVelocity());
        telemetry.update();



        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.
 /*       relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });
  */

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "LF (%.2f), RF (%.2f), LR (%.2f), RR (%.2f)", leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);
        // generic DistanceSensor methods.
      /*  telemetry.addData("deviceName",rightSensorRange.getDeviceName() );
        telemetry.addData("range", String.format("%.01f mm", rightSensorRange.getDistance(DistanceUnit.MM)));
        telemetry.addData("range", String.format("%.01f cm", rightSensorRange.getDistance(DistanceUnit.CM)));
        telemetry.addData("range", String.format("%.01f m", rightSensorRange.getDistance(DistanceUnit.METER)));
        telemetry.addData("range", String.format("%.01f in", rightSensorRange.getDistance(DistanceUnit.INCH)));
*/
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    private void initHardwareMap() {
        hardwarePushBot.mapWheels(hardwareMap);
        hardwarePushBot.mapColorSensor(hardwareMap);
        hardwarePushBot.mapRingIntake(hardwareMap);
        hardwarePushBot.mapWobbleArm(hardwareMap);
        hardwarePushBot.mapShootingWheel(hardwareMap);
        hardwarePushBot.mapTouchSensor(hardwareMap);
        hardwarePushBot.initializeImu(hardwareMap);

        hardwarePushBot.touchSensorWaFront.setMode(DigitalChannel.Mode.INPUT);

        hardwarePushBot.leftColorSensor.enableLed(true);
        hardwarePushBot.rightColorSensor.enableLed(true);

        //hardwarePushBot.setWheelDirection();
        // Note changes for Strafer Chassis below
        hardwarePushBot.leftFrontWheel.setDirection(DcMotor.Direction.FORWARD);
        hardwarePushBot.leftBackWheel.setDirection(DcMotor.Direction.FORWARD);
        hardwarePushBot.rightFrontWheel.setDirection(DcMotor.Direction.REVERSE);
        hardwarePushBot.rightBackWheel.setDirection(DcMotor.Direction.REVERSE);
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
        telemetry.addData("current Battery voltage is " , result);
        telemetry.update();
       return result;


    }

    public void startShootingWheelUsingEncoder(double shootingVelocity) {
        hardwarePushBot.shootingWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardwarePushBot.shootingWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardwarePushBot.shootingWheel.setVelocity(shootingVelocity);

    }

    


}


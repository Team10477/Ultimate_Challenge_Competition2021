package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class EncoderBot {

    double heading;
    double integralError = 0;
    double error = 0;
    double deltaTurn = 0;
    double GAIN_PROP = 0.015;
    double GAIN_INT = 0.015;
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

    public void stopResetEncoder(HardwarePushBot hardwarePushBot) {
        hardwarePushBot.leftFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardwarePushBot.leftBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardwarePushBot.rightFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardwarePushBot.rightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardwarePushBot.wobbleGoalArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void runWithoutEncoder(HardwarePushBot hardwarePushBot) {
        hardwarePushBot.leftFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardwarePushBot.leftBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardwarePushBot.rightFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardwarePushBot.rightBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void runToPositionMode(HardwarePushBot hardwarePushBot) {
        hardwarePushBot.leftFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardwarePushBot.leftBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardwarePushBot.rightFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardwarePushBot.rightBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void setBrake(HardwarePushBot hardwarePushBot) {
        hardwarePushBot.leftFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwarePushBot.rightFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwarePushBot.leftBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwarePushBot.rightBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void runToPosition(HardwarePushBot hardwarePushBot, LinearOpMode opCode, double power, Telemetry telemetry) {
        runToPositionMode(hardwarePushBot);

        hardwarePushBot.leftFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwarePushBot.rightFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwarePushBot.leftBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwarePushBot.rightBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hardwarePushBot.setWheelPower(power, power, power, power);

        while (opCode.opModeIsActive() && (hardwarePushBot.leftFrontWheel.isBusy() &&
                hardwarePushBot.rightFrontWheel.isBusy() && hardwarePushBot.leftBackWheel.isBusy() &&
                hardwarePushBot.rightBackWheel.isBusy())) {
            telemetry.addData("Current position right front", hardwarePushBot.rightFrontWheel.getCurrentPosition());
            telemetry.update();
        }
        hardwarePushBot.setWheelPower(0.0, 0.0, 0.0, 0.0);

    }

    public void runToPosition_withdistanceSensor(HardwarePushBot hardwarePushBot,  LinearOpMode opCode, double power, int distance, Telemetry telemetry) {
        runToPositionMode(hardwarePushBot);

        hardwarePushBot.leftFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwarePushBot.rightFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwarePushBot.leftBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwarePushBot.rightBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hardwarePushBot.setWheelPower(power, power, power, power);

        while (opCode.opModeIsActive() && (hardwarePushBot.leftFrontWheel.isBusy() &&
                hardwarePushBot.rightFrontWheel.isBusy() && hardwarePushBot.leftBackWheel.isBusy() &&
                hardwarePushBot.rightBackWheel.isBusy()) && hardwarePushBot.getDistance() <= distance) {
            telemetry.addData("Current position right front", hardwarePushBot.rightFrontWheel.getCurrentPosition() );
            telemetry.update();
        }
        hardwarePushBot.setWheelPower(0.0, 0.0, 0.0, 0.0);

    }

    public void runToPositionForWobbleArm(HardwarePushBot hardwarePushBot, double power, LinearOpMode opCode) {
        hardwarePushBot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hardwarePushBot.wobbleGoalArm.setPower(power);

        while (opCode.opModeIsActive() && hardwarePushBot.wobbleGoalArm.isBusy() && hardwarePushBot.touchSensorWaFront.getState()) {
            opCode.telemetry.addData("Current position wobble arm", hardwarePushBot.wobbleGoalArm.getCurrentPosition());
            opCode.telemetry.update();
        }

        hardwarePushBot.wobbleGoalArm.setPower(0);

    }

    public void runToPositionForWobbleArmBack(HardwarePushBot hardwarePushBot, double power, LinearOpMode opCode) {
        hardwarePushBot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hardwarePushBot.wobbleGoalArm.setPower(power);

        while (opCode.opModeIsActive() && hardwarePushBot.wobbleGoalArm.isBusy() && hardwarePushBot.touchSensorWaBack.getState()) {
            opCode.telemetry.addData("Current position wobble arm", hardwarePushBot.wobbleGoalArm.getCurrentPosition());
            opCode.telemetry.update();
        }

        hardwarePushBot.wobbleGoalArm.setPower(0);

    }

    public void runToPositionDiag_RF_LB(HardwarePushBot hardwarePushBot, LinearOpMode opCode){
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

        while (opCode.opModeIsActive() && (hardwarePushBot.leftBackWheel.isBusy() &&
                hardwarePushBot.rightFrontWheel.isBusy()
        )) {
            opCode.telemetry.addData("Current position right front", hardwarePushBot.rightFrontWheel.getCurrentPosition());
            opCode.telemetry.update();
        }
        hardwarePushBot.setWheelPower(0.0, 0.0, 0.0, 0.0);
    }

    public void runToPositionDiag_LF_RB(HardwarePushBot hardwarePushBot, LinearOpMode opCode){
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

        while (opCode.opModeIsActive() && (hardwarePushBot.leftFrontWheel.isBusy() &&
                hardwarePushBot.rightBackWheel.isBusy()
        )) {
            opCode.telemetry.addData("Current position right front", hardwarePushBot.leftFrontWheel.getCurrentPosition());
            opCode.telemetry.update();
        }
        hardwarePushBot.setWheelPower(0.0, 0.0, 0.0, 0.0);
    }

    public void startShootingWheelUsingEncoder(HardwarePushBot hardwarePushBot, double shootingWheelVelocity) {
        hardwarePushBot.shootingWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardwarePushBot.shootingWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardwarePushBot.shootingWheel.setVelocity(shootingWheelVelocity);

    }

    public void setTargetPosition(HardwarePushBot hardwarePushBot, int action, int position) {
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

    public void setTargetPosition_FBM(HardwarePushBot hardwarePushBot, int action, int position) {
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
            leftFrontPosition = hardwarePushBot.leftFrontWheel.getCurrentPosition() - position;
            rightBackPosition = hardwarePushBot.leftFrontWheel.getCurrentPosition() - position;
            hardwarePushBot.leftFrontWheel.setTargetPosition(leftFrontPosition);
            hardwarePushBot.rightBackWheel.setTargetPosition(rightBackPosition);
        }else if (action == 9) { //diagonal left front
            leftBackPosition = hardwarePushBot.leftBackWheel.getCurrentPosition() - position;
            rightFrontPosition = hardwarePushBot.rightFrontWheel.getCurrentPosition() - position;
            hardwarePushBot.leftBackWheel.setTargetPosition(hardwarePushBot.leftBackWheel.getCurrentPosition() - position);
            hardwarePushBot.rightFrontWheel.setTargetPosition(hardwarePushBot.rightFrontWheel.getCurrentPosition() - position);
        }else if (action == 10) { //diagonal left back
            leftFrontPosition = hardwarePushBot.leftFrontWheel.getCurrentPosition() + position;
            rightBackPosition = hardwarePushBot.leftFrontWheel.getCurrentPosition() + position;
            hardwarePushBot.leftFrontWheel.setTargetPosition(hardwarePushBot.leftFrontWheel.getCurrentPosition() + position);
            hardwarePushBot.rightBackWheel.setTargetPosition(hardwarePushBot.rightBackWheel.getCurrentPosition() + position);
        }else if (action == 11) { //diagonal right back
            leftBackPosition = hardwarePushBot.leftBackWheel.getCurrentPosition() + position;
            rightFrontPosition = hardwarePushBot.rightFrontWheel.getCurrentPosition() + position;
            hardwarePushBot.leftBackWheel.setTargetPosition(hardwarePushBot.leftBackWheel.getCurrentPosition() + position);
            hardwarePushBot.rightFrontWheel.setTargetPosition(hardwarePushBot.rightFrontWheel.getCurrentPosition() + position);

        }

    }


    public void runToPosition_FBM(HardwarePushBot hardwarePushBot, int op, LinearOpMode opCode, double power) {
        runToPositionMode(hardwarePushBot);

        hardwarePushBot.leftFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwarePushBot.rightFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwarePushBot.leftBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwarePushBot.rightBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       integralError = 0;

        while (opCode.opModeIsActive() && ((op == 1 && hardwarePushBot.leftFrontWheel.getCurrentPosition() >= leftFrontPosition && hardwarePushBot.leftBackWheel.getCurrentPosition() >= leftBackPosition
                && hardwarePushBot.rightFrontWheel.getCurrentPosition() >= rightFrontPosition && hardwarePushBot.rightBackWheel.getCurrentPosition() >= rightBackPosition) ||
                (op == 2 && hardwarePushBot.leftFrontWheel.getCurrentPosition() <= leftFrontPosition && hardwarePushBot.leftBackWheel.getCurrentPosition() <= leftBackPosition
                        && hardwarePushBot.rightFrontWheel.getCurrentPosition() <= rightFrontPosition && hardwarePushBot.rightBackWheel.getCurrentPosition() <= rightBackPosition)))  {

            heading = hardwarePushBot.getAngle();
            error = heading - 0; // desrired - current heading is the error
            integralError = integralError + error * 0.025;
            hardwarePushBot.mecanumDrive(power, 0, -(error * GAIN_PROP + integralError * GAIN_INT));
            opCode.telemetry.addData("Current position right front", hardwarePushBot.rightFrontWheel.getCurrentPosition());
            opCode.telemetry.update();
        }

        hardwarePushBot.setWheelPower(0.0, 0.0, 0.0, 0.0);


    }


    public void runToPositionDiag_LF_RB_FBM(HardwarePushBot hardwarePushBot, LinearOpMode opCode) {
        hardwarePushBot.leftFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwarePushBot.rightFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwarePushBot.leftBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwarePushBot.rightBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hardwarePushBot.leftFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardwarePushBot.leftBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardwarePushBot.rightFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardwarePushBot.rightBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        integralError = 0;

        hardwarePushBot.setWheelPower(0.0, 0.0, 0.0, 0.0);

        while (opCode.opModeIsActive() && hardwarePushBot.leftFrontWheel.getCurrentPosition() != leftFrontPosition && hardwarePushBot.rightBackWheel.getCurrentPosition() != rightBackPosition) {
            heading = hardwarePushBot.getAngle();
            error = heading - 0; // desrired - current heading is the error
            integralError = integralError + error * 0.025;
            hardwarePushBot.mecanumDrive_DiagStrafe(1, 0, -(error * GAIN_PROP + integralError * GAIN_INT), 1, 4);
            opCode.telemetry.addData("Current position right front", hardwarePushBot.rightFrontWheel.getCurrentPosition());
            opCode.telemetry.update();
        }

        hardwarePushBot.setWheelPower(0.0, 0.0, 0.0, 0.0);
    }


}

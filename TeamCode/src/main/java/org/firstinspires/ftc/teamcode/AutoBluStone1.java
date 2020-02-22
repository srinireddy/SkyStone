package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;

@Autonomous(name = "AutoBluStone1", group = "Sample")
public class AutoBluStone1 extends LinearOpMode {

    //declare motors
    private DcMotor driveFLM;
    private DcMotor driveFRM;
    private DcMotor driveBLM;
    private DcMotor driveBRM;
    private DcMotor stoneLeftM;
    private DcMotor stoneRghtM;
    private DcMotor verticalLeftM;
    private DcMotor verticalRghtM;

    //declare servos
    private Servo skystoneLeftS;
    private Servo skystoneGrabLeftS;
    private Servo waffleLeftS;
    private Servo waffleRghtS;
    private Servo succLeftS;
    private Servo succRghtS;
    private Servo stoneS;
    private Servo armS;
    private Servo clawS;

    //declare sensors of distance
    private DistanceSensor frontDS;
    private DistanceSensor rghtDS;
    private DistanceSensor backDS;

    //declare color sensor (CS) and related variables
    private ColorSensor stoneLeftCS;
    private float[] hsvValues = {0F, 0F, 0F};
    private final float[] values = hsvValues;
    private final double SCALE_FACTOR = 255;
    private int relativeLayoutId;
    private View relativeLayout;

    //declare sensor stuff
    private int robotWhere;

    @Override
    public void runOpMode() throws InterruptedException {
        //init
        if (true) {
            //change screen color
            relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
            relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

            //initialize motors
            driveFLM      = hardwareMap.dcMotor.get("driveFLM");
            driveFRM      = hardwareMap.dcMotor.get("driveFRM");
            driveBLM      = hardwareMap.dcMotor.get("driveBLM");
            driveBRM      = hardwareMap.dcMotor.get("driveBRM");
            stoneLeftM    = hardwareMap.dcMotor.get("succLeftM");
            stoneRghtM    = hardwareMap.dcMotor.get("succRightM");
            verticalLeftM = hardwareMap.dcMotor.get("verticalLeftM");
            verticalRghtM = hardwareMap.dcMotor.get("verticalRightM");

            //initialize servos
            skystoneLeftS     = hardwareMap.servo.get("skystoneLeftS");
            skystoneGrabLeftS = hardwareMap.servo.get("skystoneGrabLeftS");
            waffleLeftS       = hardwareMap.servo.get("waffleFrontS");
            waffleRghtS       = hardwareMap.servo.get("waffleBackS");
            succLeftS         = hardwareMap.servo.get("succLeftS");
            succRghtS         = hardwareMap.servo.get("succRightS");
            stoneS            = hardwareMap.servo.get("stoneS");
            armS              = hardwareMap.servo.get("slideS");
            clawS             = hardwareMap.servo.get("clawS");

            //initialize sensor of distance
            frontDS = hardwareMap.get(DistanceSensor.class, "frontDS");
            rghtDS  = hardwareMap.get(DistanceSensor.class, "rightDS");
            backDS  = hardwareMap.get(DistanceSensor.class, "backDS");

            //initialize color sensor
            stoneLeftCS = hardwareMap.colorSensor.get("skystoneLeftCS");

            //set directions
            driveFLM.setDirection(FORWARD);
            driveFRM.setDirection(REVERSE);
            driveBLM.setDirection(FORWARD);
            driveBRM.setDirection(REVERSE);
            stoneLeftM.setDirection(FORWARD);
            stoneRghtM.setDirection(REVERSE);
            verticalLeftM.setDirection(REVERSE);
            verticalRghtM.setDirection(FORWARD);

            //set to encoders
            driveFLM.setMode(RUN_USING_ENCODER);
            driveFRM.setMode(RUN_USING_ENCODER);
            driveBLM.setMode(RUN_USING_ENCODER);
            driveBRM.setMode(RUN_USING_ENCODER);
            verticalLeftM.setMode(RUN_USING_ENCODER);
            verticalRghtM.setMode(RUN_USING_ENCODER);

            //set drigve motors to brake
            driveFLM.setZeroPowerBehavior(BRAKE);
            driveFRM.setZeroPowerBehavior(BRAKE);
            driveBLM.setZeroPowerBehavior(BRAKE);
            driveBRM.setZeroPowerBehavior(BRAKE);
            verticalLeftM.setZeroPowerBehavior(BRAKE);
            verticalRghtM.setZeroPowerBehavior(BRAKE);

            telemetry.addData("ready?","Yeet");
            telemetry.update();

        }

        armS.setPosition(0.00);
        clawS.setPosition(0.15);
        succLeftS.setPosition(1.0);
        succRghtS.setPosition(0.0);
        stoneS.setPosition(0.24);

        waitForStart();

        succLeftS.setPosition(1);
        succRghtS.setPosition(0);
        waffleup();

        skystoneGrabLeftS.setPosition(0.90);
        moveLeftE(0.6, 1325);
        driveForeCS(0.3);

        if (robotWhere == 0) {
            driveForwardE(0.6, 70, false);
            liftStone();
            moveRghtE(0.8, 400);
            driveBackwardE(0.7, 2000, true);
            Thread.sleep(50);
            moveLeftE(0.5, 300);
            Thread.sleep(50);
            dropStone();
            stoneS.setPosition(0.24);

            moveRghtE(0.7, 400);
            Thread.sleep(50);
            driveForwardE(0.8, 2450, true);
            //driveForeDS(0.5, 68);
            rghtDistance(0.5, 70);
            Thread.sleep(200);
            moveLeftE(0.8, 750);
            moveRghtE(0.8, 100);
            driveBackwardE(0.8, 100, false);
            succLeftS.setPosition(0.60);
            succRghtS.setPosition(0.40);
            stoneLeftM.setPower(0.5);
            stoneRghtM.setPower(0.5);
            Thread.sleep(50);
            driveForwardE(0.25, 320, false);

            moveRghtE(0.8, 900);
            spinLeftE(0.4, 1800);
            driveForwardE(0.5, 2400, true);
            stoneLeftM.setPower(-0.7);
            stoneRghtM.setPower(-0.7);
            Thread.sleep(50);
            driveBackwardE(0.6, 500, false);
        }
        if (robotWhere == 1) {
            driveForwardE(0.6, 150, false);
            liftStone();
            moveRghtE(0.8, 400);
            driveBackwardE(0.7, 2450, true);
            Thread.sleep(50);
            moveLeftE(0.5, 300);
            Thread.sleep(50);
            dropStone();
            stoneS.setPosition(0.24);

            moveRghtE(0.7, 400);
            Thread.sleep(50);
            driveForwardE(0.8, 2900, true);
            //driveForeDS(0.5, 68);
            rghtDistance(0.5, 70);
            Thread.sleep(200);
            moveLeftE(0.8, 750);
            moveRghtE(0.8, 100);
            driveBackwardE(0.8, 100, false);
            succLeftS.setPosition(0.60);
            succRghtS.setPosition(0.40);
            stoneLeftM.setPower(0.5);
            stoneRghtM.setPower(0.5);
            Thread.sleep(50);
            driveForwardE(0.25, 320, false);

            moveRghtE(0.8, 1100);
            spinLeftE(0.4, 1800);
            driveForwardE(0.8, 2750, true);
            stoneLeftM.setPower(0.4);
            stoneRghtM.setPower(0.6);
            stoneLeftM.setPower(-0.7);
            stoneRghtM.setPower(-0.7);
            Thread.sleep(50);
            driveBackwardE(0.6, 500, false);
        }
        if (robotWhere == 2) {
            driveForwardE(0.6, 150, false);
            liftStone();
            moveRghtE(0.8, 400);
            driveBackwardE(0.7, 2700, true);
            Thread.sleep(50);
            moveLeftE(0.5, 300);
            Thread.sleep(50);
            dropStone();
            stoneS.setPosition(0.24);

            moveRghtE(0.7, 400);
            Thread.sleep(50);
            driveForwardE(0.8, 3050, true);
            //driveForeDS(0.5, 68);
            rghtDistance(0.5, 70);
            Thread.sleep(200);
            moveLeftE(0.8, 750);
            moveRghtE(0.8, 100);
            driveBackwardE(0.8, 100, false);
            succLeftS.setPosition(0.60);
            succRghtS.setPosition(0.40);
            stoneLeftM.setPower(0.5);
            stoneRghtM.setPower(0.5);
            Thread.sleep(50);
            driveForwardE(0.25, 320, false);

            moveRghtE(0.8, 1100);
            spinLeftE(0.4, 1800);
            driveForwardE(0.8, 3100, true);
            stoneLeftM.setPower(0.4);
            stoneRghtM.setPower(0.6);
            stoneLeftM.setPower(-0.7);
            stoneRghtM.setPower(-0.7);
            Thread.sleep(50);
            driveBackwardE(0.6, 500, false);
        }

    }
    //methods
    private void liftStone() throws InterruptedException {
        skystoneLeftS.setPosition(0.7);
        Thread.sleep(200);
        skystoneGrabLeftS.setPosition(0.7);
        skystoneLeftS.setPosition(1.0);
        Thread.sleep(200);
        skystoneGrabLeftS.setPosition(0.0);
        Thread.sleep(600);
        skystoneLeftS.setPosition(0.8);
        skystoneLeftS.setPosition(0.6);
        skystoneLeftS.setPosition(0.4);
        skystoneLeftS.setPosition(0.2);
        skystoneLeftS.setPosition(0.0);
        Thread.sleep(300);
    }
    private void dropStone() throws InterruptedException {
        skystoneGrabLeftS.setPosition(0.0);
        Thread.sleep(100);
        skystoneLeftS.setPosition(0.4);
        Thread.sleep(400);
        skystoneGrabLeftS.setPosition(0.7);
        Thread.sleep(100);
        skystoneLeftS.setPosition(0.0);
        skystoneGrabLeftS.setPosition(1.0);
    }

    private void waffledown() {
        waffleLeftS.setPosition(0.35);
        waffleRghtS.setPosition(0.62);
    }
    private void waffleup() {
        waffleLeftS.setPosition(0.00);
        waffleRghtS.setPosition(1.00);
    }

    private void verticalSlideUp(double power, int ticks) {
        verticalRghtM.setTargetPosition(ticks);
        verticalRghtM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalLeftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalLeftM.setPower(power);
        verticalRghtM.setPower(power);
        while (verticalRghtM.isBusy()) {}
        verticalLeftM.setPower(0);
        verticalRghtM.setPower(0);
        verticalLeftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalRghtM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void verticalSlideDown(double power, int ticks) {
        verticalRghtM.setTargetPosition(ticks);
        verticalRghtM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalLeftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalLeftM.setPower(-power);
        verticalRghtM.setPower(-power);
        while (verticalRghtM.isBusy()) {}
        verticalLeftM.setPower(0);
        verticalRghtM.setPower(0);
        verticalLeftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalRghtM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void driveForwardE(double power, int ticks, boolean rampDown) {
        //Reset Encoders
        driveFLM.setMode(STOP_AND_RESET_ENCODER);
        driveFRM.setMode(STOP_AND_RESET_ENCODER);
        driveBLM.setMode(STOP_AND_RESET_ENCODER);
        driveBRM.setMode(STOP_AND_RESET_ENCODER);

        //set target position
        driveFLM.setTargetPosition(ticks);
        driveFRM.setTargetPosition(ticks);
        driveBLM.setTargetPosition(ticks);
        driveBRM.setTargetPosition(ticks);

        //set ot RUN_TO_POSITION mode
        driveFLM.setMode(RUN_TO_POSITION);
        driveFRM.setMode(RUN_TO_POSITION);
        driveBLM.setMode(RUN_TO_POSITION);
        driveBRM.setMode(RUN_TO_POSITION);

        //driveForward
        driveFLM.setPower(power);
        driveFRM.setPower(power);
        driveBLM.setPower(power);
        driveBRM.setPower(power);

        if (rampDown) {
            //wait until target position
            while (driveFLM.isBusy() && driveFRM.isBusy() && driveBLM.isBusy() && driveBRM.isBusy()) {
                if (Math.abs(driveFLM.getCurrentPosition()-driveFLM.getTargetPosition())<350) {
                    driveFLM.setPower(0.5);
                    driveFRM.setPower(0.5);
                    driveBLM.setPower(0.5);
                    driveBRM.setPower(0.5);
                }
            }
        }else {
            //wait until target position
            while (driveFLM.isBusy() && driveFRM.isBusy() && driveBLM.isBusy() && driveBRM.isBusy()) { }
        }

        //stopMoving();
        driveFLM.setPower(0);
        driveFRM.setPower(0);
        driveBLM.setPower(0);
        driveBRM.setPower(0);

        driveFLM.setZeroPowerBehavior(BRAKE);
        driveFRM.setZeroPowerBehavior(BRAKE);
        driveBLM.setZeroPowerBehavior(BRAKE);
        driveBRM.setZeroPowerBehavior(BRAKE);

        driveFLM.setMode(RUN_USING_ENCODER);
        driveFRM.setMode(RUN_USING_ENCODER);
        driveBLM.setMode(RUN_USING_ENCODER);
        driveBRM.setMode(RUN_USING_ENCODER);
    }
    private void driveBackwardE(double power, int ticks, boolean rampDown) {
        //Reset Encoders
        driveFLM.setMode(STOP_AND_RESET_ENCODER);
        driveFRM.setMode(STOP_AND_RESET_ENCODER);
        driveBLM.setMode(STOP_AND_RESET_ENCODER);
        driveBRM.setMode(STOP_AND_RESET_ENCODER);

        //set target position
        driveFLM.setTargetPosition(-ticks);
        driveFRM.setTargetPosition(-ticks);
        driveBLM.setTargetPosition(-ticks);
        driveBRM.setTargetPosition(-ticks);

        //set ot RUN_TO_POSITION mode
        driveFLM.setMode(RUN_TO_POSITION);
        driveFRM.setMode(RUN_TO_POSITION);
        driveBLM.setMode(RUN_TO_POSITION);
        driveBRM.setMode(RUN_TO_POSITION);

        //driveBackward
        driveFLM.setPower(power);
        driveFRM.setPower(power);
        driveBLM.setPower(power);
        driveBRM.setPower(power);

        if (rampDown) {
            //wait until target position
            while (driveFLM.isBusy() && driveFRM.isBusy() && driveBLM.isBusy() && driveBRM.isBusy()) {
                if (Math.abs(driveFLM.getCurrentPosition()-driveFLM.getTargetPosition())<350) {
                    driveFLM.setPower(0.5);
                    driveFRM.setPower(0.5);
                    driveBLM.setPower(0.5);
                    driveBRM.setPower(0.5);
                }
            }
        }else {
            //wait until target position
            while (driveFLM.isBusy() && driveFRM.isBusy() && driveBLM.isBusy() && driveBRM.isBusy()) { }
        }

        //stopMoving();
        driveFLM.setPower(0);
        driveFRM.setPower(0);
        driveBLM.setPower(0);
        driveBRM.setPower(0);

        driveFLM.setZeroPowerBehavior(BRAKE);
        driveFRM.setZeroPowerBehavior(BRAKE);
        driveBLM.setZeroPowerBehavior(BRAKE);
        driveBRM.setZeroPowerBehavior(BRAKE);

        driveFLM.setMode(RUN_USING_ENCODER);
        driveFRM.setMode(RUN_USING_ENCODER);
        driveBLM.setMode(RUN_USING_ENCODER);
        driveBRM.setMode(RUN_USING_ENCODER);
    }
    private void spinLeftE(double power, int ticks) {
        //Reset Encoders
        driveFLM.setMode(STOP_AND_RESET_ENCODER);
        driveFRM.setMode(STOP_AND_RESET_ENCODER);
        driveBLM.setMode(STOP_AND_RESET_ENCODER);
        driveBRM.setMode(STOP_AND_RESET_ENCODER);

        //set target position
        driveFLM.setTargetPosition(-ticks);
        driveFRM.setTargetPosition(ticks);
        driveBLM.setTargetPosition(-ticks);
        driveBRM.setTargetPosition(ticks);

        //set ot RUN_TO_POSITION mode
        driveFLM.setMode(RUN_TO_POSITION);
        driveFRM.setMode(RUN_TO_POSITION);
        driveBLM.setMode(RUN_TO_POSITION);
        driveBRM.setMode(RUN_TO_POSITION);

        //spinLeft
        driveFLM.setPower(power);
        driveFRM.setPower(power);
        driveBLM.setPower(power);
        driveBRM.setPower(power);

        //wait until target position
        while (driveFLM.isBusy() && driveFRM.isBusy() && driveBLM.isBusy() && driveBRM.isBusy()) {}

        //stopMoving();
        driveFLM.setPower(0);
        driveFRM.setPower(0);
        driveBLM.setPower(0);
        driveBRM.setPower(0);

        driveFLM.setZeroPowerBehavior(BRAKE);
        driveFRM.setZeroPowerBehavior(BRAKE);
        driveBLM.setZeroPowerBehavior(BRAKE);
        driveBRM.setZeroPowerBehavior(BRAKE);

        driveFLM.setMode(RUN_USING_ENCODER);
        driveFRM.setMode(RUN_USING_ENCODER);
        driveBLM.setMode(RUN_USING_ENCODER);
        driveBRM.setMode(RUN_USING_ENCODER);
    }
    private void spinRghtE(double power, int ticks) {
        //Reset Encoders
        driveFLM.setMode(STOP_AND_RESET_ENCODER);
        driveFRM.setMode(STOP_AND_RESET_ENCODER);
        driveBLM.setMode(STOP_AND_RESET_ENCODER);
        driveBRM.setMode(STOP_AND_RESET_ENCODER);

        //set target position
        driveFLM.setTargetPosition(ticks);
        driveFRM.setTargetPosition(-ticks);
        driveBLM.setTargetPosition(ticks);
        driveBRM.setTargetPosition(-ticks);

        //set ot RUN_TO_POSITION mode
        driveFLM.setMode(RUN_TO_POSITION);
        driveFRM.setMode(RUN_TO_POSITION);
        driveBLM.setMode(RUN_TO_POSITION);
        driveBRM.setMode(RUN_TO_POSITION);

        //spinRight
        driveFLM.setPower(power);
        driveFRM.setPower(power);
        driveBLM.setPower(power);
        driveBRM.setPower(power);

        //wait until target position
        while (driveFLM.isBusy() && driveFRM.isBusy() && driveBLM.isBusy() && driveBRM.isBusy()) {}

        //stopMoving();
        driveFLM.setPower(0);
        driveFRM.setPower(0);
        driveBLM.setPower(0);
        driveBRM.setPower(0);

        driveFLM.setZeroPowerBehavior(BRAKE);
        driveFRM.setZeroPowerBehavior(BRAKE);
        driveBLM.setZeroPowerBehavior(BRAKE);
        driveBRM.setZeroPowerBehavior(BRAKE);

        driveFLM.setMode(RUN_USING_ENCODER);
        driveFRM.setMode(RUN_USING_ENCODER);
        driveBLM.setMode(RUN_USING_ENCODER);
        driveBRM.setMode(RUN_USING_ENCODER);
    }
    private void moveLeftE(double power, int ticks) {
        //Reset Encoders
        driveFLM.setMode(STOP_AND_RESET_ENCODER);
        driveFRM.setMode(STOP_AND_RESET_ENCODER);
        driveBLM.setMode(STOP_AND_RESET_ENCODER);
        driveBRM.setMode(STOP_AND_RESET_ENCODER);

        //set target position
        driveFLM.setTargetPosition(-ticks);
        driveFRM.setTargetPosition(ticks);
        driveBLM.setTargetPosition(ticks);
        driveBRM.setTargetPosition(-ticks);

        //set ot RUN_TO_POSITION mode
        driveFLM.setMode(RUN_TO_POSITION);
        driveFRM.setMode(RUN_TO_POSITION);
        driveBLM.setMode(RUN_TO_POSITION);
        driveBRM.setMode(RUN_TO_POSITION);

        //moveLeft
        driveFLM.setPower(power);
        driveFRM.setPower(power);
        driveBLM.setPower(power);
        driveBRM.setPower(power);

        //wait until target position
        while (driveFLM.isBusy() && driveFRM.isBusy() && driveBLM.isBusy() && driveBRM.isBusy()) {}

        //stopMoving();
        driveFLM.setPower(0);
        driveFRM.setPower(0);
        driveBLM.setPower(0);
        driveBRM.setPower(0);

        driveFLM.setZeroPowerBehavior(BRAKE);
        driveFRM.setZeroPowerBehavior(BRAKE);
        driveBLM.setZeroPowerBehavior(BRAKE);
        driveBRM.setZeroPowerBehavior(BRAKE);

        driveFLM.setMode(RUN_USING_ENCODER);
        driveFRM.setMode(RUN_USING_ENCODER);
        driveBLM.setMode(RUN_USING_ENCODER);
        driveBRM.setMode(RUN_USING_ENCODER);
    }
    private void moveRghtE(double power, int ticks) {
        //Reset Encoders
        driveFLM.setMode(STOP_AND_RESET_ENCODER);
        driveFRM.setMode(STOP_AND_RESET_ENCODER);
        driveBLM.setMode(STOP_AND_RESET_ENCODER);
        driveBRM.setMode(STOP_AND_RESET_ENCODER);

        //set target position
        driveFLM.setTargetPosition(ticks);
        driveFRM.setTargetPosition(-ticks);
        driveBLM.setTargetPosition(-ticks);
        driveBRM.setTargetPosition(ticks);

        //set ot RUN_TO_POSITION mode
        driveFLM.setMode(RUN_TO_POSITION);
        driveFRM.setMode(RUN_TO_POSITION);
        driveBLM.setMode(RUN_TO_POSITION);
        driveBRM.setMode(RUN_TO_POSITION);

        //moveRight
        driveFLM.setPower(power);
        driveFRM.setPower(power);
        driveBLM.setPower(power);
        driveBRM.setPower(power);

        //wait until target position
        while (driveFLM.isBusy() && driveFRM.isBusy() && driveBLM.isBusy() && driveBRM.isBusy()) {}

        //stopMoving();
        driveFLM.setPower(0);
        driveFRM.setPower(0);
        driveBLM.setPower(0);
        driveBRM.setPower(0);

        driveFLM.setZeroPowerBehavior(BRAKE);
        driveFRM.setZeroPowerBehavior(BRAKE);
        driveBLM.setZeroPowerBehavior(BRAKE);
        driveBRM.setZeroPowerBehavior(BRAKE);

        driveFLM.setMode(RUN_USING_ENCODER);
        driveFRM.setMode(RUN_USING_ENCODER);
        driveBLM.setMode(RUN_USING_ENCODER);
        driveBRM.setMode(RUN_USING_ENCODER);
    }

    private void turnLmoveFE(double power, int ticks, double multiplier) {
        //Reset Encoders
        driveFLM.setMode(STOP_AND_RESET_ENCODER);
        driveFRM.setMode(STOP_AND_RESET_ENCODER);
        driveBLM.setMode(STOP_AND_RESET_ENCODER);
        driveBRM.setMode(STOP_AND_RESET_ENCODER);

        //set target position
        driveFLM.setTargetPosition(ticks);
        driveFRM.setTargetPosition(ticks);
        driveBLM.setTargetPosition(ticks);
        driveBRM.setTargetPosition(ticks);

        //set ot RUN_TO_POSITION mode
        driveFLM.setMode(RUN_TO_POSITION);
        driveFRM.setMode(RUN_TO_POSITION);
        driveBLM.setMode(RUN_TO_POSITION);
        driveBRM.setMode(RUN_TO_POSITION);

        //driveForward
        driveFLM.setPower(power);
        driveFRM.setPower(power*multiplier);
        driveBLM.setPower(power);
        driveBRM.setPower(power*multiplier);

        //wait until target position
        while (driveFRM.isBusy() && driveBRM.isBusy())
        {

        }

        //stopMoving();
        driveFLM.setPower(0);
        driveFRM.setPower(0);
        driveBLM.setPower(0);
        driveBRM.setPower(0);

        driveFLM.setZeroPowerBehavior(BRAKE);
        driveFRM.setZeroPowerBehavior(BRAKE);
        driveBLM.setZeroPowerBehavior(BRAKE);
        driveBRM.setZeroPowerBehavior(BRAKE);

        driveFLM.setMode(RUN_USING_ENCODER);
        driveFRM.setMode(RUN_USING_ENCODER);
        driveBLM.setMode(RUN_USING_ENCODER);
        driveBRM.setMode(RUN_USING_ENCODER);
    }

    private void rghtDistance(double power, double distance) {
        telemetry.addData("distance", rghtDS.getDistance(CM));
        telemetry.update();
        if (rghtDS.getDistance(CM) >= distance) {
            driveFLM.setPower(power);
            driveFRM.setPower(-power);
            driveBLM.setPower(-power);
            driveBRM.setPower(power);

            while (rghtDS.getDistance(CM) > distance) {
                telemetry.addData("distance", rghtDS.getDistance(CM));
                telemetry.update();
            }

            driveFLM.setPower(0);
            driveFRM.setPower(0);
            driveBLM.setPower(0);
            driveBRM.setPower(0);
        }else {
            driveFLM.setPower(-power);
            driveFRM.setPower(power);
            driveBLM.setPower(power);
            driveBRM.setPower(-power);

            while (rghtDS.getDistance(CM) < distance) {
                telemetry.addData("distance", rghtDS.getDistance(CM));
                telemetry.update();
            }

            driveFLM.setPower(0);
            driveFRM.setPower(0);
            driveBLM.setPower(0);
            driveBRM.setPower(0);
        }
    }
    private void driveForeDS(double power, double wallDistance) {
        telemetry.addData("distance", frontDS.getDistance(DistanceUnit.CM));
        telemetry.update();
        if (frontDS.getDistance(DistanceUnit.CM) >= wallDistance) {
            driveFLM.setPower(power);
            driveFRM.setPower(power);
            driveBLM.setPower(power);
            driveBRM.setPower(power);

            while (frontDS.getDistance(DistanceUnit.CM) > wallDistance) {
                telemetry.addData("distance", frontDS.getDistance(DistanceUnit.CM));
                telemetry.update();
            }

            driveFLM.setPower(0);
            driveFRM.setPower(0);
            driveBLM.setPower(0);
            driveBRM.setPower(0);
        }else {
            driveFLM.setPower(-power);
            driveFRM.setPower(-power);
            driveBLM.setPower(-power);
            driveBRM.setPower(-power);

            while (frontDS.getDistance(DistanceUnit.CM) < wallDistance) {
                telemetry.addData("distance", frontDS.getDistance(DistanceUnit.CM));
                telemetry.update();
            }

            driveFLM.setPower(0);
            driveFRM.setPower(0);
            driveBLM.setPower(0);
            driveBRM.setPower(0);
        }
    }
    
    private void driveForeCS(double power) {
        driveFLM.setMode(STOP_AND_RESET_ENCODER);
        driveFRM.setMode(STOP_AND_RESET_ENCODER);
        driveBLM.setMode(STOP_AND_RESET_ENCODER);
        driveBRM.setMode(STOP_AND_RESET_ENCODER);

        driveFLM.setMode(RUN_USING_ENCODER);
        driveFRM.setMode(RUN_USING_ENCODER);
        driveBLM.setMode(RUN_USING_ENCODER);
        driveBRM.setMode(RUN_USING_ENCODER);

        Color.RGBToHSV(
                (int) (stoneLeftCS.red() * SCALE_FACTOR),
                (int) (stoneLeftCS.green() * SCALE_FACTOR),
                (int) (stoneLeftCS.blue() * SCALE_FACTOR),
                hsvValues);

        //telemetry
        telemetry.addData("Hoo", hsvValues[0]);
        telemetry.addData("Satchurashun", hsvValues[1]);
        telemetry.addData("Valyoo", hsvValues[2]);
        telemetry.update();

        if (hsvValues[0]>90) {
            robotWhere = 0;
            return;
        }
        driveFLM.setPower(power);
        driveFRM.setPower(power);
        driveBLM.setPower(power);
        driveBRM.setPower(power);

        while (hsvValues[0]<90) {
            Color.RGBToHSV(
                    (int) (stoneLeftCS.red() * SCALE_FACTOR),
                    (int) (stoneLeftCS.green() * SCALE_FACTOR),
                    (int) (stoneLeftCS.blue() * SCALE_FACTOR),
                    hsvValues);

            //telemetry
            telemetry.addData("Hoo", hsvValues[0]);
            telemetry.addData("Satchurashun", hsvValues[1]);
            telemetry.addData("Valyoo", hsvValues[2]);
            telemetry.update();

            driveFLM.setPower(power);
            driveFRM.setPower(power);
            driveBLM.setPower(power);
            driveBRM.setPower(power);
        }

        driveFLM.setPower(0);
        driveFRM.setPower(0);
        driveBLM.setPower(0);
        driveBRM.setPower(0);

        if (driveFLM.getCurrentPosition()<50) {
            robotWhere=0;
        }else {
            if (driveFLM.getCurrentPosition()<350) {
                robotWhere=1;
            }else {
                robotWhere = 2;
            }
        }

        driveFLM.setMode(RUN_USING_ENCODER);
        driveFRM.setMode(RUN_USING_ENCODER);
        driveBLM.setMode(RUN_USING_ENCODER);
        driveBRM.setMode(RUN_USING_ENCODER);
    }
}

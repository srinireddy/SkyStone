package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.util.Log;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;

@Autonomous(name = "AutoRedBoth0", group = "Sample")
public class AutoRedBoth0 extends LinearOpMode {

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
    private Servo skystoneRghtS;
    private Servo skystoneGrabRghtS;
    private Servo waffleLeftS;
    private Servo waffleRghtS;
    private Servo succLeftS;
    private Servo succRghtS;
    private Servo stoneS;
    private Servo armS;
    private Servo clawS;

    //declare sensors of distance
    private DistanceSensor frontDS;
    private DistanceSensor leftDS;
    private DistanceSensor backDS;

    //declare color sensor (CS) and related variables
    private ColorSensor stoneRghtCS;
    float[] hsvValues = {0F, 0F, 0F};
    final float[] values = hsvValues;
    final double SCALE_FACTOR = 255;
    int relativeLayoutId;
    View relativeLayout;

    //declare skystone stuaagg stuff
    private int robotWhere;

    @Override
    public void runOpMode() throws InterruptedException {
        //init
        if (true) {
            //change screen color
            relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
            relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
            
            //initialize motors
            driveFLM = hardwareMap.dcMotor.get("driveFLM");
            driveFRM = hardwareMap.dcMotor.get("driveFRM");
            driveBLM = hardwareMap.dcMotor.get("driveBLM");
            driveBRM = hardwareMap.dcMotor.get("driveBRM");
            stoneLeftM = hardwareMap.dcMotor.get("succLeftM");
            stoneRghtM = hardwareMap.dcMotor.get("succRightM");
            verticalLeftM = hardwareMap.dcMotor.get("verticalLeftM");
            verticalRghtM = hardwareMap.dcMotor.get("verticalRightM");

            //initialize servos
            skystoneRghtS     = hardwareMap.servo.get("skystoneRightS");
            skystoneGrabRghtS = hardwareMap.servo.get("skystoneGrabRightS");
            waffleLeftS       = hardwareMap.servo.get("waffleFrontS");
            waffleRghtS       = hardwareMap.servo.get("waffleBackS");
            succLeftS         = hardwareMap.servo.get("succLeftS");
            succRghtS         = hardwareMap.servo.get("succRightS");
            stoneS            = hardwareMap.servo.get("stoneS");
            armS              = hardwareMap.servo.get("slideS");
            clawS             = hardwareMap.servo.get("clawS");

            //initialize sensor of distance
            frontDS = hardwareMap.get(DistanceSensor.class, "frontDS");
            leftDS  = hardwareMap.get(DistanceSensor.class, "leftDS");
            backDS  = hardwareMap.get(DistanceSensor.class, "backDS");

            //initialize color sensor
            stoneRghtCS = hardwareMap.colorSensor.get("skystoneRightCS");

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

            //set drive motors to brake
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
        waffleup();

        waitForStart();

        succLeftS.setPosition(1);
        succRghtS.setPosition(0);
        waffleup();

        skystoneGrabRghtS.setPosition(0.10);
        moveRghtE(0.6, 1285);
        driveForeCS(0.3);

        if (robotWhere == 0) {
            driveForwardE(0.6, 70, false);
            liftStone();
            moveLeftE(0.8, 350);
            driveBackwardE(0.7, 2700, true);
            Thread.sleep(50);
            Thread.sleep(50);
            leftDistance(0.3, 79);
            dropStone();
            stoneS.setPosition(0.10);

            moveLeftE(0.7, 400);
            Thread.sleep(50);
            driveForwardE(0.8, 3150, true);
            //driveForeDS(0.5, 68);
            leftDistance(0.5, 70);
            Thread.sleep(200);
            moveRghtE(0.8, 750);
            moveLeftE(0.8, 100);
            driveBackwardE(0.8, 100, false);
            succLeftS.setPosition(0.60);
            succRghtS.setPosition(0.40);
            stoneLeftM.setPower(0.5);
            stoneRghtM.setPower(0.5);
            Thread.sleep(50);
            driveForwardE(0.25, 320, false);

            moveLeftE(0.8, 1100);
            driveBackwardE(0.8, 3300, true);
            Thread.sleep(50);

            spinLeftE(0.8, 900);
            driveForeDS(0.5, 75);
            waffledown();
            Thread.sleep(500);
            stoneLeftM.setPower(-0.7);
            stoneRghtM.setPower(-0.7);
            spinRghtE(0.9, 200);
            driveForwardE(0.9, 400, false);
            turnRmoveFE(0.12, 3100, 7);
            driveBackwardE(0.8, 800, false);
            driveBackwardE(0.2, 200, false);

            waffleup();
            Thread.sleep(500);
            leftDistance(0.7, 60);
            stoneLeftM.setPower(-0.7);
            stoneRghtM.setPower(-0.7);
            driveForwardE(0.5, 1300, false);
        }
        if (robotWhere == 1) {
            driveForwardE(0.6, 150, false);
            liftStone();
            moveLeftE(0.8, 350);
            driveBackwardE(0.8, 3150, true);
            Thread.sleep(50);
            Thread.sleep(50);
            leftDistance(0.3, 79);
            dropStone();
            stoneS.setPosition(0.10);

            moveLeftE(0.7, 400);
            Thread.sleep(50);
            driveForwardE(0.8, 3600, true);
            //driveForeDS(0.5, 63);
            leftDistance(0.5, 70);
            Thread.sleep(200);
            moveRghtE(0.8, 750);
            moveLeftE(0.8, 100);
            driveBackwardE(0.8, 100, false);
            succLeftS.setPosition(0.60);
            succRghtS.setPosition(0.40);
            stoneLeftM.setPower(0.5);
            stoneRghtM.setPower(0.5);
            Thread.sleep(50);
            driveForwardE(0.25, 320, false);

            moveLeftE(0.8, 1100);
            driveBackwardE(0.8, 3800, true);
            Thread.sleep(50);

            spinLeftE(0.8, 900);
            driveForeDS(0.5, 75);
            waffledown();
            Thread.sleep(500);
            stoneLeftM.setPower(-0.7);
            stoneRghtM.setPower(-0.7);
            spinRghtE(0.9, 200);
            driveForwardE(0.9, 400, false);
            turnRmoveFE(0.12, 3100, 7);
            driveBackwardE(0.8, 800, false);
            driveBackwardE(0.2, 200, false);

            waffleup();
            Thread.sleep(500);
            leftDistance(0.7, 60);
            stoneLeftM.setPower(-0.7);
            stoneRghtM.setPower(-0.7);
            driveForwardE(0.5, 1300, false);
        }
        if (robotWhere == 2) {
            driveForwardE(0.6, 150, false);
            liftStone();
            moveLeftE(0.8, 350);
            driveBackwardE(0.8, 3400, true);
            Thread.sleep(50);
            Thread.sleep(50);
            leftDistance(0.3, 79);
            dropStone();
            stoneS.setPosition(0.10);

            moveLeftE(0.7, 400);
            Thread.sleep(50);
            driveForwardE(0.8, 3750, true);
            //driveForeDS(0.5, 43);
            leftDistance(0.5, 70);
            Thread.sleep(200);
            moveRghtE(0.8, 750);
            moveLeftE(0.8, 100);
            driveBackwardE(0.8, 100, false);
            succLeftS.setPosition(0.60);
            succRghtS.setPosition(0.40);
            stoneLeftM.setPower(0.5);
            stoneRghtM.setPower(0.5);
            Thread.sleep(50);
            driveForwardE(0.25, 320, false);

            moveLeftE(0.8, 1100);
            driveBackwardE(0.8, 4050, true);
            Thread.sleep(50);

            spinLeftE(0.8, 900);
            driveForeDS(0.5, 75);
            waffledown();
            Thread.sleep(500);
            stoneLeftM.setPower(-0.7);
            stoneRghtM.setPower(-0.7);
            spinRghtE(0.9, 200);
            driveForwardE(0.9, 400, false);
            turnRmoveFE(0.12, 3100, 7);
            driveBackwardE(0.8, 800, false);
            driveBackwardE(0.2, 200, false);

            waffleup();
            Thread.sleep(500);
            leftDistance(0.7, 60);
            driveForwardE(0.5, 1300, false);
        }
    }
    //methods
    private void liftStone() throws InterruptedException {
        skystoneRghtS.setPosition(0.3);
        Thread.sleep(200);
        skystoneGrabRghtS.setPosition(0.3);
        skystoneRghtS.setPosition(0.0);
        Thread.sleep(200);
        skystoneGrabRghtS.setPosition(1.0);
        Thread.sleep(600);
        skystoneRghtS.setPosition(0.2);
        skystoneRghtS.setPosition(0.4);
        skystoneRghtS.setPosition(0.6);
        skystoneRghtS.setPosition(0.8);
        skystoneRghtS.setPosition(1.0);
        Thread.sleep(300);
    }
    private void dropStone() throws InterruptedException {
        skystoneGrabRghtS.setPosition(1.0);
        Thread.sleep(100);
        skystoneRghtS.setPosition(0.6);
        Thread.sleep(400);
        skystoneGrabRghtS.setPosition(0.3);
        Thread.sleep(100);
        skystoneRghtS.setPosition(1.0);
        skystoneGrabRghtS.setPosition(0.0);
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

    private void turnRmoveFE(double power, int ticks, double multiplier) {
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
        driveFLM.setPower(power*multiplier);
        driveFRM.setPower(power);
        driveBLM.setPower(power*multiplier);
        driveBRM.setPower(power);

        //wait until target position
        while (driveFLM.isBusy() && driveBLM.isBusy())
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
    
    private void leftDistance(double power, double distance) {
        telemetry.addData("distance", leftDS.getDistance(CM));
        telemetry.update();
        if (leftDS.getDistance(CM) >= distance) {
            driveFLM.setPower(-power);
            driveFRM.setPower(power);
            driveBLM.setPower(power);
            driveBRM.setPower(-power);

            while (leftDS.getDistance(CM) > distance) {
                telemetry.addData("distance", leftDS.getDistance(CM));
                telemetry.update();
            }

            driveFLM.setPower(0);
            driveFRM.setPower(0);
            driveBLM.setPower(0);
            driveBRM.setPower(0);
        }else {
            driveFLM.setPower(power);
            driveFRM.setPower(-power);
            driveBLM.setPower(-power);
            driveBRM.setPower(power);

            while (leftDS.getDistance(CM) < distance) {
                telemetry.addData("distance", leftDS.getDistance(CM));
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

        driveFLM.setPower(power);
        driveFRM.setPower(power);
        driveBLM.setPower(power);
        driveBRM.setPower(power);

        Color.RGBToHSV(
                (int) (stoneRghtCS.red() * SCALE_FACTOR),
                (int) (stoneRghtCS.green() * SCALE_FACTOR),
                (int) (stoneRghtCS.blue() * SCALE_FACTOR),
                hsvValues);

        //telemetry
        telemetry.addData("Hoo", hsvValues[0]);
        telemetry.addData("Satchurashun", hsvValues[1]);
        telemetry.addData("Valyoo", hsvValues[2]);
        telemetry.update();

        while (hsvValues[0]<90) {
            Color.RGBToHSV(
                    (int) (stoneRghtCS.red() * SCALE_FACTOR),
                    (int) (stoneRghtCS.green() * SCALE_FACTOR),
                    (int) (stoneRghtCS.blue() * SCALE_FACTOR),
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

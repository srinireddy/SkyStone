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
import static java.lang.Enum.valueOf;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;

@Autonomous(name = "AutoBluBoth0", group = "Sample")
public class AutoBluBoth0 extends LinearOpMode {

    //declare motors
    private DcMotor driveFLM;
    private DcMotor driveFRM;
    private DcMotor driveBLM;
    private DcMotor driveBRM;
    private DcMotor stoneLeftM;
    private DcMotor stoneRghtM;

    //declare servos
    private Servo skystoneLeftS;
    private Servo skystoneGrabLeftS;
    private Servo waffleLeftS;
    private Servo waffleRghtS;
    private Servo succLeftS;
    private Servo succRghtS;
    private Servo stoneS;
    private Servo slideS;

    //declare sensors of distance
    private DistanceSensor frontDS;
    private DistanceSensor rghtDS;
    private DistanceSensor backDS;
    private DistanceSensor rightCS;

    // The IMU sensor object
    private BNO055IMU imu;
    private Orientation angles;
    private Orientation lastAngles = new Orientation();
    private double globalAngle;

    //declare color sensor (CS) and related variables
    private ColorSensor stoneLeftCS;
    float[] hsvValues = {0F, 0F, 0F};
    final float[] values = hsvValues;
    final double SCALE_FACTOR = 255;
    int relativeLayoutId;
    View relativeLayout;

    //declare sensor stuff
    private boolean isSkystone = false;
    private int tries = 0;
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

            //initialize servos
            skystoneLeftS     = hardwareMap.servo.get("skystoneLeftS");
            skystoneGrabLeftS = hardwareMap.servo.get("skystoneGrabLeftS");
            waffleLeftS       = hardwareMap.servo.get("waffleFrontS");
            waffleRghtS       = hardwareMap.servo.get("waffleBackS");
            succLeftS         = hardwareMap.servo.get("succLeftS");
            succRghtS         = hardwareMap.servo.get("succRightS");
            stoneS            = hardwareMap.servo.get("stoneS");
            slideS            = hardwareMap.servo.get("slideS");

            //initialize sensor of distance
            frontDS = hardwareMap.get(DistanceSensor.class, "frontDS");
            rghtDS  = hardwareMap.get(DistanceSensor.class, "rightDS");
            backDS  = hardwareMap.get(DistanceSensor.class, "backDS");
            rightCS = hardwareMap.get(DistanceSensor.class, "skystoneRightCS");

            //initialize color sensor
            stoneLeftCS = hardwareMap.colorSensor.get("skystoneLeftCS");

            //set directions
            driveFLM.setDirection(FORWARD);
            driveFRM.setDirection(REVERSE);
            driveBLM.setDirection(FORWARD);
            driveBRM.setDirection(REVERSE);
            stoneLeftM.setDirection(FORWARD);
            stoneRghtM.setDirection(REVERSE);

            //set to encoders
            driveFLM.setMode(RUN_USING_ENCODER);
            driveFRM.setMode(RUN_USING_ENCODER);
            driveBLM.setMode(RUN_USING_ENCODER);
            driveBRM.setMode(RUN_USING_ENCODER);

            //set drigve motors to brake
            driveFLM.setZeroPowerBehavior(BRAKE);
            driveFRM.setZeroPowerBehavior(BRAKE);
            driveBLM.setZeroPowerBehavior(BRAKE);
            driveBRM.setZeroPowerBehavior(BRAKE);

            //gyro stuff
            /*BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json";
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);*/

            telemetry.addData("ready?","Yeet");
            telemetry.update();

        }

        waitForStart();

        succLeftS.setPosition(1);
        succRghtS.setPosition(0);
        waffleup();

        skystoneGrabLeftS.setPosition(0.90);
        moveLeftE(0.6, 1290);
        driveForeCS(0.3);

        if (robotWhere == 0) {
            driveForwardE(0.6, 70, false);
            liftStone();
            moveRghtE(0.8, 300);
            driveBackwardE(0.8, 2700, true);
            Thread.sleep(50);
            moveLeftE(0.5, 500);
            Thread.sleep(50);
            rghtDistance(0.3, 79);
            dropStone();
            stoneS.setPosition(0.10);

            moveRghtE(0.7, 400);
            Thread.sleep(50);
            driveForwardE(0.8, 3150, true);
            driveForeDS(0.5, 83);
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
            driveForwardE(0.3, 320, false);

            succLeftS.setPosition(0.9);
            succRghtS.setPosition(0.1);
            stoneLeftM.setPower(0);
            stoneRghtM.setPower(0);
            moveRghtE(0.8, 1100);
            stoneS.setPosition(1);
            Thread.sleep(50);
            driveBackwardE(0.8, 3500, true);
            Thread.sleep(50);

            spinRghtE(0.8, 900);
            driveForeDS(0.5, 75);
            driveBackwardE(0.4, 300, false);
            waffledown();
            Thread.sleep(500);
            spinLeftE(0.9, 200);
            driveForwardE(0.9, 400, false);
            turnLmoveFE(0.12, 3100, 7);
            driveBackwardE(0.8, 800, false);
            driveBackwardE(0.2, 200, false);

            waffleup();
            Thread.sleep(500);
            rghtDistance(0.7, 65    );
            slideS.setPosition(0.0);
            succLeftS.setPosition(0.50);
            succRghtS.setPosition(0.50);
            driveForwardE(0.5, 1300, false);
            slideS.setPosition(0.5);
        }
        if (robotWhere == 1) {
            driveForwardE(0.6, 150, false);
            liftStone();
            moveRghtE(0.8, 300);
            driveBackwardE(0.8, 3050, true);
            Thread.sleep(50);
            moveLeftE(0.5, 200);
            Thread.sleep(50);
            rghtDistance(0.3, 79);
            dropStone();
            stoneS.setPosition(0.10);

            moveRghtE(0.7, 400);
            Thread.sleep(50);
            driveForwardE(0.8, 3400, true);
            driveForeDS(0.5, 63);
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
            driveForwardE(0.3, 320, false);

            succLeftS.setPosition(0.9);
            succRghtS.setPosition(0.1);
            stoneLeftM.setPower(0);
            stoneRghtM.setPower(0);
            moveRghtE(0.8, 1100);
            stoneS.setPosition(1);
            Thread.sleep(50);
            driveBackwardE(0.8, 3800, true);
            Thread.sleep(50);

            spinRghtE(0.8, 900);
            driveForeDS(0.5, 75);
            driveBackwardE(0.4, 300, false);
            waffledown();
            Thread.sleep(500);
            spinLeftE(0.9, 200);
            driveForwardE(0.9, 400, false);
            turnLmoveFE(0.12, 3100, 7);
            driveBackwardE(0.8, 800, false);
            driveBackwardE(0.2, 200, false);

            waffleup();
            Thread.sleep(500);
            rghtDistance(0.7, 65);
            slideS.setPosition(0.0);
            succLeftS.setPosition(0.50);
            succRghtS.setPosition(0.50);
            driveForwardE(0.5, 1300, false);
            slideS.setPosition(0.5);
        }
        if (robotWhere == 2) {
            driveForwardE(0.6, 150, false);
            liftStone();
            moveRghtE(0.8, 300);
            driveBackwardE(0.8, 3400, true);
            Thread.sleep(50);
            moveLeftE(0.5, 200);
            Thread.sleep(50);
            rghtDistance(0.3, 79);
            dropStone();
            stoneS.setPosition(0.10);

            moveRghtE(0.7, 400);
            Thread.sleep(50);
            driveForwardE(0.8, 3750, true);
            driveForeDS(0.5, 43);
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
            driveForwardE(0.3, 320, false);

            succLeftS.setPosition(0.9);
            succRghtS.setPosition(0.1);
            stoneLeftM.setPower(0);
            stoneRghtM.setPower(0);
            moveRghtE(0.8, 1100);
            stoneS.setPosition(1);
            Thread.sleep(50);
            driveBackwardE(0.8, 4150, true);
            Thread.sleep(50);

            spinRghtE(0.8, 900);
            driveForeDS(0.5, 75);
            driveBackwardE(0.4, 300, false);
            waffledown();
            Thread.sleep(500);
            spinLeftE(0.9, 200);
            driveForwardE(0.9, 400, false);
            turnLmoveFE(0.12, 3100, 7);
            driveBackwardE(0.8, 800, false);
            driveBackwardE(0.2, 200, false);

            waffleup();
            Thread.sleep(500);
            rghtDistance(0.7, 65);
            slideS.setPosition(0.0);
            succLeftS.setPosition(0.50);
            succRghtS.setPosition(0.50);
            driveForwardE(0.5, 1300, false);
            slideS.setPosition(0.5);
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
        while (driveFLM.isBusy() && driveFRM.isBusy() && driveBLM.isBusy() && driveBRM.isBusy())
        {
            telemetry.addData("distance2", rightCS.getDistance(CM));
            telemetry.update();
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
        while (driveFLM.isBusy() && driveFRM.isBusy() && driveBLM.isBusy() && driveBRM.isBusy())
        {
            telemetry.addData("distance2", rightCS.getDistance(CM));
            telemetry.update();
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
        while (driveFLM.isBusy() && driveFRM.isBusy() && driveBLM.isBusy() && driveBRM.isBusy())
        {
            telemetry.addData("distance2", rightCS.getDistance(CM));
            telemetry.update();
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
        while (driveFLM.isBusy() && driveFRM.isBusy() && driveBLM.isBusy() && driveBRM.isBusy())
        {
            telemetry.addData("distance2", rightCS.getDistance(CM));
            telemetry.update();
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

        driveFLM.setPower(power);
        driveFRM.setPower(power);
        driveBLM.setPower(power);
        driveBRM.setPower(power);

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
    private void getSkystone() throws InterruptedException {

        //changes to HSV instead of RGB cuz it's more useful
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

        //changes color of the RC
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });

        //says we found the skystone.
        if (hsvValues[0]>90) {isSkystone = true;}
    }
    private void checkStones() throws InterruptedException {
        //basically the same structure as last year
        while (!isSkystone&&tries<2) {
            tries++;
            robotWhere = 0;
            getSkystone();
            if (!isSkystone) {
                driveForwardE(0.3, 350, false);
                robotWhere = 1;
                getSkystone();
                if (!isSkystone) {
                    driveForwardE(0.3, 350, false);
                    robotWhere = 2;
                    getSkystone();
                    if (!isSkystone) {
                        driveBackwardE(0.3, 700, false);
                        robotWhere = 0;
                    }
                }
            }
        }
    }
}

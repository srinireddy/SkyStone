package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;

@Autonomous(name = "AutoRedWaffle2", group = "Sample")
public class AutoRedWaffle2 extends LinearOpMode {

    //declare motors
    private DcMotor driveFLM;
    private DcMotor driveFRM;
    private DcMotor driveBLM;
    private DcMotor driveBRM;
    private DcMotor stoneLeftM;
    private DcMotor stoneRghtM;

    //declare servos
    private Servo skystoneRghtS;
    private Servo skystoneGrabRghtS;
    private Servo waffleLeftS;
    private Servo waffleRghtS;
    private Servo succLeftS;
    private Servo succRghtS;

    //declare sensors of distance
    private DistanceSensor frontDS;
    private DistanceSensor leftDS;
    private DistanceSensor backDS;
    private DistanceSensor rightCS;

    // The IMU sensor object
    private BNO055IMU imu;
    private Orientation angles;
    private Orientation lastAngles = new Orientation();
    private double globalAngle;

    @Override
    public void runOpMode() throws InterruptedException {
        //init
        if (true) {
            //initialize motors
            driveFLM = hardwareMap.dcMotor.get("driveFLM");
            driveFRM = hardwareMap.dcMotor.get("driveFRM");
            driveBLM = hardwareMap.dcMotor.get("driveBLM");
            driveBRM = hardwareMap.dcMotor.get("driveBRM");
            stoneLeftM = hardwareMap.dcMotor.get("succLeftM");
            stoneRghtM = hardwareMap.dcMotor.get("succRightM");

            //initialize servos
            skystoneRghtS     = hardwareMap.servo.get("skystoneRightS");
            skystoneGrabRghtS = hardwareMap.servo.get("skystoneGrabRightS");
            waffleLeftS       = hardwareMap.servo.get("waffleFrontS");
            waffleRghtS       = hardwareMap.servo.get("waffleBackS");
            succLeftS         = hardwareMap.servo.get("succLeftS");
            succRghtS         = hardwareMap.servo.get("succRightS");

            //initialize sensor of distance
            frontDS = hardwareMap.get(DistanceSensor.class, "frontDS");
            leftDS  = hardwareMap.get(DistanceSensor.class, "leftBackDS");
            backDS  = hardwareMap.get(DistanceSensor.class, "backDS");
            rightCS = hardwareMap.get(DistanceSensor.class, "skystoneRightCS");

            //set directions
            driveFLM.setDirection(DcMotor.Direction.FORWARD);
            driveFRM.setDirection(DcMotor.Direction.REVERSE);
            driveBLM.setDirection(DcMotor.Direction.FORWARD);
            driveBRM.setDirection(DcMotor.Direction.REVERSE);
            stoneLeftM.setDirection(DcMotor.Direction.FORWARD);
            stoneRghtM.setDirection(DcMotor.Direction.REVERSE);

            //set to encoders
            driveFLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveFRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveBLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveBRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //set drigve motors to brake
            driveFLM.setZeroPowerBehavior(BRAKE);
            driveFRM.setZeroPowerBehavior(BRAKE);
            driveBLM.setZeroPowerBehavior(BRAKE);
            driveBRM.setZeroPowerBehavior(BRAKE);

            telemetry.addData("ready?","Yeet");
            telemetry.update();

        }

        waitForStart();

        succLeftS.setPosition(1);
        succRghtS.setPosition(0);
        waffleup();
        driveBackwardE(1.0, 100);
        moveLeftE(1.0, 450);
        driveBackwardE(0.6, 1160);
        driveBackwardE(0.4, 200);
        waffledown();

        Thread.sleep(500);
        spinRghtE(0.9, 200);
        driveForwardE(0.9, 400);
        turnRmoveFE(0.12, 3100, 7);
        driveBackwardE(0.8, 1000);
        driveBackwardE(0.2, 300);
        waffleup();
        Thread.sleep(500);
        moveLeftE(0.7, 500);
        driveForwardE(0.5, 1700);
        moveLeftE(0.3, 400);

    }
    //methods
    private void waffledown() {
        waffleLeftS.setPosition(0.35);
        waffleRghtS.setPosition(0.62);
    }
    private void waffleup() {
        waffleLeftS.setPosition(0.00);
        waffleRghtS.setPosition(1.00);
    }

    private void driveForwardE(double power, int ticks) {
        //Reset Encoders
        driveFLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        driveFLM.setTargetPosition(ticks);
        driveFRM.setTargetPosition(ticks);
        driveBLM.setTargetPosition(ticks);
        driveBRM.setTargetPosition(ticks);

        //set ot RUN_TO_POSITION mode
        driveFLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveFRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //driveForward
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

        driveFLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void driveBackwardE(double power, int ticks) {
        //Reset Encoders
        driveFLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        driveFLM.setTargetPosition(-ticks);
        driveFRM.setTargetPosition(-ticks);
        driveBLM.setTargetPosition(-ticks);
        driveBRM.setTargetPosition(-ticks);

        //set ot RUN_TO_POSITION mode
        driveFLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveFRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //driveBackward
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

        driveFLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void spinLeftE(double power, int ticks) {
        //Reset Encoders
        driveFLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        driveFLM.setTargetPosition(-ticks);
        driveFRM.setTargetPosition(ticks);
        driveBLM.setTargetPosition(-ticks);
        driveBRM.setTargetPosition(ticks);

        //set ot RUN_TO_POSITION mode
        driveFLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveFRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

        driveFLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void spinRghtE(double power, int ticks) {
        //Reset Encoders
        driveFLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        driveFLM.setTargetPosition(ticks);
        driveFRM.setTargetPosition(-ticks);
        driveBLM.setTargetPosition(ticks);
        driveBRM.setTargetPosition(-ticks);

        //set ot RUN_TO_POSITION mode
        driveFLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveFRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

        driveFLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void moveLeftE(double power, int ticks) {
        //Reset Encoders
        driveFLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        driveFLM.setTargetPosition(-ticks);
        driveFRM.setTargetPosition(ticks);
        driveBLM.setTargetPosition(ticks);
        driveBRM.setTargetPosition(-ticks);

        //set ot RUN_TO_POSITION mode
        driveFLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveFRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

        driveFLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void moveRghtE(double power, int ticks) {
        //Reset Encoders
        driveFLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        driveFLM.setTargetPosition(ticks);
        driveFRM.setTargetPosition(-ticks);
        driveBLM.setTargetPosition(-ticks);
        driveBRM.setTargetPosition(ticks);

        //set ot RUN_TO_POSITION mode
        driveFLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveFRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

        driveFLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void turnRmoveFE(double power, int ticks, double multiplier) {
        //Reset Encoders
        driveFLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        driveFLM.setTargetPosition(ticks);
        driveFRM.setTargetPosition(ticks);
        driveBLM.setTargetPosition(ticks);
        driveBRM.setTargetPosition(ticks);

        //set ot RUN_TO_POSITION mode
        driveFLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveFRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

        driveFLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

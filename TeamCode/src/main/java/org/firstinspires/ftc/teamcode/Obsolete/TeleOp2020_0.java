package org.firstinspires.ftc.teamcode.Obsolete;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "TeleOp2020_0", group = "Sample")
@Disabled
public class TeleOp2020_0 extends LinearOpMode{

    //declare motors
    private DcMotor driveFLM;
    private DcMotor driveFRM;
    private DcMotor driveBLM;
    private DcMotor driveBRM;
    private DcMotor stoneLeftM;
    private DcMotor stoneRghtM;
    private DcMotor verticalLeftM;
    private DcMotor verticalRghtM;

    //declare swervos
    private Servo succLeftS;
    private Servo succRghtS;
    private Servo armLeftS;
    private Servo armRghtS;
    private Servo stoneLeftS;
    private Servo stoneRghtS;
    private  Servo stoneS;

    //declare distance sensor/detector
    private DistanceSensor stoneDS;

    double drivespeed = 0.5;
    boolean stoned = false;

    @Override
    public void
    runOpMode() {

        //configuration
        if (true) {
            //configure motors
            driveFLM      = hardwareMap.dcMotor.get("driveFLM");
            driveFRM      = hardwareMap.dcMotor.get("driveFRM");
            driveBLM      = hardwareMap.dcMotor.get("driveBLM");
            driveBRM      = hardwareMap.dcMotor.get("driveBRM");
            stoneLeftM    = hardwareMap.dcMotor.get("stoneLeftM");
            stoneRghtM    = hardwareMap.dcMotor.get("stoneRightM");
            verticalLeftM = hardwareMap.dcMotor.get("verticalLeftM");
            verticalRghtM = hardwareMap.dcMotor.get("verticalRightM");

            //configure swervos
            succLeftS     = hardwareMap.servo.get("succLeftS");
            succRghtS     = hardwareMap.servo.get("succRightS");
           /* armLeftS      = hardwareMap.servo.get("armLeftS");
            armRghtS      = hardwareMap.servo.get("armRightS");
            stoneLeftS    = hardwareMap.servo.get("stoneLeftS");
            stoneRghtS    = hardwareMap.servo.get("stoneRightS");
            stoneS        = hardwareMap.servo.get("stoneS");*/

            //configure distance detector
            stoneDS = hardwareMap.get(DistanceSensor.class, "stoneDS");

            //set motor directions
            driveFLM.setDirection(DcMotor.Direction.FORWARD);
            driveFRM.setDirection(DcMotor.Direction.REVERSE);
            driveBLM.setDirection(DcMotor.Direction.FORWARD);
            driveBRM.setDirection(DcMotor.Direction.REVERSE);
            stoneLeftM.setDirection(DcMotor.Direction.REVERSE);
            stoneRghtM.setDirection(DcMotor.Direction.FORWARD);
            verticalLeftM.setDirection(DcMotor.Direction.FORWARD);
            verticalRghtM.setDirection(DcMotor.Direction.REVERSE);

            //set continuous servo directions
          /* armLeftS.setDirection(Servo.Direction.FORWARD);
            armRghtS.setDirection(Servo.Direction.REVERSE);
            stoneLeftS.setDirection(Servo.Direction.FORWARD);
            stoneRghtS.setDirection(Servo.Direction.REVERSE);*/

            //set all motors to not use encoders
            driveFLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveFRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveBLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveBRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            stoneLeftM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            stoneRghtM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            verticalLeftM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            verticalRghtM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        waitForStart();

        //move succ motors out at start of teleop
        succLeftS.setPosition(0.5);
        succRghtS.setPosition(0.5);

        while(opModeIsActive()) {

            if (gamepad1.dpad_down) {
                drivespeed=0.5;
            }else {
                if (gamepad1.dpad_up) {
                    drivespeed=1;
                }
            }

            //lets KV move the robot
            //left stick moves the robot without turning, it is driving and strafing
            //right stick x for spinning
            driveFLM.setPower((-gamepad1.left_stick_y+gamepad1.left_stick_x+gamepad1.right_stick_x)*drivespeed);
            driveFRM.setPower((-gamepad1.left_stick_y-gamepad1.left_stick_x-gamepad1.right_stick_x)*drivespeed);
            driveBLM.setPower((-gamepad1.left_stick_y-gamepad1.left_stick_x+gamepad1.right_stick_x)*drivespeed);
            driveBRM.setPower((-gamepad1.left_stick_y+gamepad1.left_stick_x-gamepad1.right_stick_x)*drivespeed);

            //KV can start and stop the succ motors
            if (gamepad1.a) {
                stoneLeftM.setPower(1);
                stoneRghtM.setPower(1);
                stoned = true;
            }else {
                if (gamepad1.b) {
                    stoneLeftM.setPower(0);
                    stoneRghtM.setPower(0);
                }else {
                    if (gamepad1.y) {
                        stoneLeftM.setPower(-1);
                        stoneRghtM.setPower(-1);
                    }
                }
            }

            if (gamepad2.right_bumper) {
                verticalLeftM.setPower(0.5);
                verticalRghtM.setPower(0.5);
            }else {
                if (gamepad2.left_bumper) {
                    verticalLeftM.setPower(-0.5);
                    verticalRghtM.setPower(-0.5);
                }else {
                    verticalLeftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    verticalRghtM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    verticalLeftM.setPower(0);
                    verticalRghtM.setPower(0);
                    verticalLeftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    verticalRghtM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }

            //KV can move the servos holding the succ motors in and out
            if (gamepad1.right_bumper) {
                succLeftS.setPosition(0.5);
                succRghtS.setPosition(0.5);
            }else {
                if (gamepad1.left_bumper) {
                    succLeftS.setPosition(1);
                    succRghtS.setPosition(0);
                }else {
                    if (gamepad1.right_trigger>0.1) {
                        succLeftS.setPosition(0.25);
                        succRghtS.setPosition(0.75);
                    }else {
                        if (stoneDS.getDistance(DistanceUnit.CM) < 10 && stoned) {
                            stoneLeftM.setPower(0);
                            stoneRghtM.setPower(0);
                            succLeftS.setPosition(1);
                            succRghtS.setPosition(0);
                            stoned = false;
                        }
                    }
                }
            }
/*
            if (gamepad2.dpad_up) {
                armLeftS.setPosition(0.75);
                armRghtS.setPosition(0.75);
            }else {
                if (gamepad2.dpad_down) {
                    armLeftS.setPosition(0.25);
                    armRghtS.setPosition(0.25);
                }else {
                    armLeftS.setPosition(.5);
                    armRghtS.setPosition(.5);
                }
            }

            //Mike uses the triggers to move just the servos at the end of the arm.
            //he uses this to help place stone and to keep it horizontal.
            if (gamepad2.right_trigger>0.1) {
                stoneLeftS.setPosition(0.65);
                stoneRghtS.setPosition(0.65);
            }else {
                if (gamepad2.left_trigger>0.1) {
                    stoneLeftS.setPosition(0.35);
                    stoneRghtS.setPosition(0.35);
                }else {
                    stoneLeftS.setPosition(0.5);
                    stoneRghtS.setPosition(0.5);
                }
            }

            //Mike grabs or lets go of the stone at the end of the arm
            if (gamepad2.b) {
                stoneS.setPosition(0);
            }else {
                if (gamepad2.a) {
                    stoneS.setPosition(1);
                }
            }
*/
        }
    }
}

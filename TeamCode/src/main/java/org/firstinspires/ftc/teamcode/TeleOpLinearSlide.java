package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

@TeleOp(name = "teleop       linear                slide", group = "Sample")
public class TeleOpLinearSlide extends LinearOpMode{

    //declare motors
    private DcMotor driveFLM;
    private DcMotor driveFRM;
    private DcMotor driveBLM;
    private DcMotor driveBRM;
    private DcMotor succLeftM;
    private DcMotor succRghtM;
    private DcMotor verticalLeftM;
    private DcMotor verticalRghtM;

    //declare swervos
    private Servo succLeftS;
    private Servo succRghtS;
    private Servo slideS;
    private Servo stoneS;
    private Servo skystoneLeftS;
    private Servo skystoneRghtS;
    private Servo skystoneGrabLeftS;
    private Servo skystoneGrabRghtS;
    private Servo waffleForeS;
    private Servo waffleBackS;
    private Servo flipperS;

    //sensor of color
    private ColorSensor ColorSensorOfStone;
    private float[] hsvValues = {0F, 0F, 0F};
    final float[] values = hsvValues;
    
    //epic things keeping track of things
    private double driveSpeed = 0.6;
    private boolean flipperUp = false;
    private boolean stoned = false;
    
    //staging stuff
    private boolean stopDown = false;
    private double stageNum = 0;
    private double stageTarget;
    private int stageTicks;
    private boolean isStagingUp = false;
    private boolean isStagingDown = false;

    //more staging stuff
    private final int stage0 = 0;
    private final int stage1 = 600;
    private final int stage2 = 1080;
    private final int stage3 = 1550;
    private final int stage4 = 2040;
    private final int stage5 = 2460;
    private final int stage6 = 2950;


    @Override
    public void
    runOpMode() throws InterruptedException {

        //configuration
        if (true) {
            //configure motors
            driveFLM      = hardwareMap.dcMotor.get("driveFLM");
            driveFRM      = hardwareMap.dcMotor.get("driveFRM");
            driveBLM      = hardwareMap.dcMotor.get("driveBLM");
            driveBRM      = hardwareMap.dcMotor.get("driveBRM");
            succLeftM     = hardwareMap.dcMotor.get("succLeftM");
            succRghtM     = hardwareMap.dcMotor.get("succRightM");
            verticalLeftM = hardwareMap.dcMotor.get("verticalLeftM");
            verticalRghtM = hardwareMap.dcMotor.get("verticalRightM");

            //configure swervos
            succLeftS         = hardwareMap.servo.get("succLeftS");
            succRghtS         = hardwareMap.servo.get("succRightS");
            slideS            = hardwareMap.servo.get("slideS");
            stoneS            = hardwareMap.servo.get("stoneS");
            skystoneLeftS     = hardwareMap.servo.get("skystoneLeftS");
            skystoneRghtS     = hardwareMap.servo.get("skystoneRightS");
            skystoneGrabLeftS = hardwareMap.servo.get("skystoneGrabLeftS");
            skystoneGrabRghtS = hardwareMap.servo.get("skystoneGrabRightS");
            waffleForeS       = hardwareMap.servo.get("waffleFrontS");
            waffleBackS       = hardwareMap.servo.get("waffleBackS");
            flipperS          = hardwareMap.servo.get("flipperS");
            
            //color sensor initialize
            //ColorSensorOfStone = hardwareMap.colorSensor.get("stoneCS");

            //set motor directions
            driveFLM.setDirection(DcMotor.Direction.FORWARD);
            driveFRM.setDirection(DcMotor.Direction.REVERSE);
            driveBLM.setDirection(DcMotor.Direction.FORWARD);
            driveBRM.setDirection(DcMotor.Direction.REVERSE);
            succLeftM.setDirection(DcMotor.Direction.FORWARD);
            succRghtM.setDirection(DcMotor.Direction.REVERSE);
            verticalLeftM.setDirection(DcMotorSimple.Direction.REVERSE);
            verticalRghtM.setDirection(DcMotorSimple.Direction.FORWARD);

            //set most motors to not use encoders
            driveFLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveFRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveBLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveBRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            succLeftM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            succRghtM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //use encoders for this slidey boi
            verticalLeftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalRghtM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalLeftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            verticalRghtM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            verticalLeftM.setZeroPowerBehavior(BRAKE);
            verticalRghtM.setZeroPowerBehavior(BRAKE);
        }

        waitForStart();
        
        //reset encoders
        verticalLeftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRghtM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLeftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalRghtM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        //move servos to start position
        succLeftS.setPosition(0.60);
        succRghtS.setPosition(0.40);
        skystoneRghtS.setPosition(1.0);
        skystoneLeftS.setPosition(0.0);
        waffleForeS.setPosition(0.00);
        waffleBackS.setPosition(1.00);

        while(opModeIsActive()) {
            telemetry.addData("encoder data right", verticalRghtM.getCurrentPosition());
            telemetry.addData("stage number uwu", stageNum);
            telemetry.update();

            //Color.RGBToHSV((ColorSensorOfStone.red()*255), (ColorSensorOfStone.green()*255), (ColorSensorOfStone.blue()*255), hsvValues);

            //KV can change how fast he drives.
            if (gamepad1.left_stick_button) {
                driveSpeed = 0.25;
            } else {
                if (gamepad1.right_stick_button) {
                    driveSpeed = 0.6;
                }
            }

            //lets KV move the robot
            //left stick moves the robot without turning, it is driving and strafing
            //right stick x for spinning
            driveFLM.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * driveSpeed);
            driveFRM.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * driveSpeed);
            driveBLM.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * driveSpeed);
            driveBRM.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * driveSpeed);

            //KV can start, stop, and reverse the succ motors
            if (gamepad1.a) {
                succLeftM.setPower(0.7);
                succRghtM.setPower(0.7);
                stoned = true;
            } else {
                if (gamepad1.b) {
                    succLeftM.setPower(0);
                    succRghtM.setPower(0);
                } else {
                    if (gamepad1.y) {
                        succLeftM.setPower(-0.7);
                        succRghtM.setPower(-0.7);
                    }
                }
            }

            //KV can move the servos holding the succ motors in and out
            //if the stone sensor sees the stone, at 10 cm, it'll stop the succ motors and move them in
            //'stoned' boolean is basically whether the sensor is active or not
            //so that KV can move the succ motors out and the sensor wont keep trying to pull it in
            if (gamepad1.right_bumper) {
                succLeftS.setPosition(0.60);
                succRghtS.setPosition(0.40);
            } else {
                if (gamepad1.left_bumper) {
                    succLeftS.setPosition(1);
                    succRghtS.setPosition(0);
                } else {
                    if (gamepad1.right_trigger > 0.1) {
                        succLeftS.setPosition(0.25);
                        succRghtS.setPosition(0.75);
                    }else {
                        /*if (stoned && hsvValues[0]<90) {
                            succLeftM.setPower(0);
                            succRghtM.setPower(0);
                            succLeftS.setPosition(1);
                            succRghtS.setPosition(0);
                            stoned = false;
                        }*/
                    }
                }
            }

            //kv can move the waffle swervos
            if (gamepad1.dpad_up) {
                waffleForeS.setPosition(0.00);
                waffleBackS.setPosition(1.00);
            }else {
                if (gamepad1.dpad_down) {
                    waffleForeS.setPosition(0.35);
                    waffleBackS.setPosition(0.62);
                }
            }

            if (gamepad1.x) {
                flipperUp = !flipperUp;
            }
            if (flipperUp) flipperS.setPosition(0);
            if (!flipperUp) flipperS.setPosition(1);

            //if (gamepad2.back) MikeMode = !MikeMode;
            

            if (gamepad2.right_bumper) {
                verticalLeftM.setPower(0.9);
                verticalRghtM.setPower(0.9);
            }else {
                if (gamepad2.left_bumper) {
                    verticalLeftM.setPower(-1.0);
                    verticalRghtM.setPower(-1.0);
                }else {
                    verticalLeftM.setZeroPowerBehavior(BRAKE);
                    verticalRghtM.setZeroPowerBehavior(BRAKE);
                    verticalLeftM.setPower(0);
                    verticalRghtM.setPower(0);
                    verticalLeftM.setZeroPowerBehavior(BRAKE);
                    verticalRghtM.setZeroPowerBehavior(BRAKE);
                }
            }
             
            if (gamepad2.a) stoneS.setPosition(1.0);
            if (gamepad2.b) stoneS.setPosition(0.24);

            slideS.setPosition((gamepad2.left_stick_x+1)*0.5);

            if (gamepad2.x) liftStone();
            if (gamepad2.y) dropStone();

            /**all this crap below is staging and automatic stuff for the vertical claw*/
            //go down all the way
            if (gamepad2.left_stick_button) {
                    verticalRghtM.setTargetPosition(0);
                    verticalLeftM.setTargetPosition(0);
                    verticalRghtM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    verticalLeftM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    verticalLeftM.setPower(-1.00);
                    verticalRghtM.setPower(-1.00);
                    
                    while (verticalRghtM.isBusy() && !stopDown) {
                        if (gamepad2.right_stick_button) {
                            stopDown = true;
                        }
                    }
                    
                    stopDown = false;
                    verticalLeftM.setPower(0);
                    verticalRghtM.setPower(0);
                    verticalLeftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    verticalRghtM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }

            //find the stage number
            if (true) {
               if (verticalRghtM.getCurrentPosition() <= 50) stageNum = 0.0;
               if (verticalRghtM.getCurrentPosition() > stage0+50) stageNum = 0.5;
               if (verticalRghtM.getCurrentPosition() > stage1-50) stageNum = 1.0;
               if (verticalRghtM.getCurrentPosition() > stage1+50) stageNum = 1.5;
               if (verticalRghtM.getCurrentPosition() > stage2-50) stageNum = 2.0;
               if (verticalRghtM.getCurrentPosition() > stage2+50) stageNum = 2.5;
               if (verticalRghtM.getCurrentPosition() > stage3-50) stageNum = 3.0;
               if (verticalRghtM.getCurrentPosition() > stage3+50) stageNum = 3.5;
               if (verticalRghtM.getCurrentPosition() > stage4-50) stageNum = 4.0;
               if (verticalRghtM.getCurrentPosition() > stage4+50) stageNum = 4.5;
               if (verticalRghtM.getCurrentPosition() > stage5-50) stageNum = 5.0;
               if (verticalRghtM.getCurrentPosition() > stage5+50) stageNum = 5.5;
               if (verticalRghtM.getCurrentPosition() > stage6-50) stageNum = 6.0;
               if (verticalRghtM.getCurrentPosition() > stage6+50) stageNum = 6.5;
           }

            //when he says to stage up or down, it finds what stage he wants to go to
            if (gamepad2.dpad_down && stageNum>0) {
                stageTarget=Math.floor(stageNum-0.1);
                isStagingDown=true;
            }
            if (gamepad2.dpad_up && stageNum<6) {
                stageTarget=Math.ceil(stageNum+0.1);
                isStagingUp=true;
            }

            telemetry.update();

            //finds what tick value to go to for each stage
            if (true) {
                if (stageTarget==0) stageTicks=stage0;
                if (stageTarget==1) stageTicks=stage1;
                if (stageTarget==2) stageTicks=stage2;
                if (stageTarget==3) stageTicks=stage3;
                if (stageTarget==4) stageTicks=stage4;
                if (stageTarget==5) stageTicks=stage5;
                if (stageTarget==6) stageTicks=stage6;
            }

            //actually move, up or down
            if (isStagingDown) {
                verticalRghtM.setTargetPosition(stageTicks);
                verticalRghtM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                verticalLeftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                verticalLeftM.setPower(-0.75);
                verticalRghtM.setPower(-0.75);

                while (verticalRghtM.isBusy() && !stopDown) {
                    if (gamepad2.right_stick_button) {
                        stopDown = true;
                    }
                }
                stopDown = false;
                isStagingDown = false;
                verticalLeftM.setPower(0);
                verticalRghtM.setPower(0);
                verticalLeftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                verticalRghtM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (isStagingUp) {
                verticalRghtM.setTargetPosition(stageTicks);
                verticalRghtM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                verticalLeftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                verticalLeftM.setPower(0.75);
                verticalRghtM.setPower(0.75);

                while (verticalRghtM.isBusy() && !stopDown) {
                    if (gamepad2.right_stick_button) {
                        stopDown = true;
                    }
                }
                stopDown = false;
                isStagingUp = false;
                verticalLeftM.setPower(0);
                verticalRghtM.setPower(0);
                verticalLeftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                verticalRghtM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }
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
}

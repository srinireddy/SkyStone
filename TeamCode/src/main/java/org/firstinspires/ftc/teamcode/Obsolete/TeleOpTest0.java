package org.firstinspires.ftc.teamcode.Obsolete;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOpTest0", group = "Sample")
@Disabled
public class TeleOpTest0 extends LinearOpMode {
    //declare
    private DcMotor driveFLM;
    private DcMotor driveFRM;
    private DcMotor driveBLM;
    private DcMotor driveBRM;
    private DcMotor armM;
    private DcMotor stoneLeftM;
    private DcMotor stoneRghtM;

    private Servo stoneS;
    private Servo armS;
    private Servo stoneLeftS;
    private Servo stoneRghtS;

    @Override
    public void
    runOpMode() throws InterruptedException {
        //init
        driveFLM   = hardwareMap.dcMotor.get("driveFLM");
        driveFRM   = hardwareMap.dcMotor.get("driveFRM");
        driveBLM   = hardwareMap.dcMotor.get("driveBLM");
        driveBRM   = hardwareMap.dcMotor.get("driveBRM");
        armM       = hardwareMap.dcMotor.get("armM");
        stoneLeftM = hardwareMap.dcMotor.get("stoneLeftM");
        stoneRghtM = hardwareMap.dcMotor.get("stoneRghtM");

        stoneS     = hardwareMap.servo.get("stoneS");
        armS       = hardwareMap.servo.get("armS");
        stoneLeftS = hardwareMap.servo.get("stoneLeftS");
        stoneRghtS = hardwareMap.servo.get("stoneRghtS");

        driveFLM.setDirection(DcMotor.Direction.FORWARD);
        driveFRM.setDirection(DcMotor.Direction.REVERSE);
        driveBLM.setDirection(DcMotor.Direction.FORWARD);
        driveBRM.setDirection(DcMotor.Direction.REVERSE);
        armM.setDirection(DcMotor.Direction.FORWARD);
        stoneLeftM.setDirection(DcMotor.Direction.REVERSE);
        stoneRghtM.setDirection(DcMotor.Direction.FORWARD);

        driveFLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stoneLeftM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stoneRghtM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        //what runs
        while(opModeIsActive()) {

            //lets KV move the robot
            //left stick moves the robot without turning, it is driving and strafing
            //right stick x for spinning
            driveFLM.setPower((-gamepad1.left_stick_y+gamepad1.left_stick_x+gamepad1.right_stick_x)*.1);
            driveFRM.setPower((-gamepad1.left_stick_y-gamepad1.left_stick_x-gamepad1.right_stick_x)*.1);
            driveBLM.setPower((-gamepad1.left_stick_y-gamepad1.left_stick_x+gamepad1.right_stick_x)*.1);
            driveBRM.setPower((-gamepad1.left_stick_y+gamepad1.left_stick_x-gamepad1.right_stick_x)*.1);

            if (gamepad2.left_trigger>0.1) {
                armM.setPower(1);
                armS.setPosition(0.7);
            }else{
                if (gamepad2.right_trigger>0.1) {
                    armM.setPower(-1);
                    armS.setPosition(0.3);
                } else{
                    armM.setPower(0);
                    armS.setPosition(0.5);
                }
            }

            if (gamepad2.a) {
                stoneS.setPosition(1);
            } else{
                if (gamepad2.b) {
                    stoneS.setPosition(0.5);
                }
            }

            if (gamepad1.a) {
                stoneLeftM.setPower(1);
                stoneRghtM.setPower(1);
            }else{
                if (gamepad1.b) {
                    stoneLeftM.setPower(0);
                    stoneRghtM.setPower(0);
                }
            }

            stoneLeftS.setPosition(0.45);
            stoneRghtS.setPosition(0.55);
        }
    }
}

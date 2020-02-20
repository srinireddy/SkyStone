package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "GenericAutonomous", group = "Sample")
@Disabled
public class test_pulley extends LinearOpMode {

    @Override
    public void runOpMode() {

        //init
        DcMotor pulley  = hardwareMap.dcMotor.get("pulley");
        DcMotor pulley2 = hardwareMap.dcMotor.get("pulley2");
        Servo   armS = hardwareMap.servo.get("armS");

        waitForStart();
        while (opModeIsActive()) {
            //what runs
            if (gamepad1.right_bumper) {
                pulley.setPower(0.5);
                pulley2.setPower(0.5);
            } else {
                if (gamepad1.left_bumper) {
                    pulley.setPower(-0.5);
                    pulley2.setPower(-0.5);
                }else {
                    pulley.setPower(0);
                    pulley2.setPower(0);
                }
            }

            if (gamepad1.a) {
                armS.setPosition(0.2);
            }else {
                if (gamepad1.b) {
                    armS.setPosition(0.8);
                }
            }
        }
    }
}

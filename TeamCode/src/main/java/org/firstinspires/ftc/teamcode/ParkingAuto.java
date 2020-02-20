package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
@Autonomous(name="Park",group="Sample")
public class ParkingAuto extends LinearOpMode{
    @Override public void runOpMode()throws InterruptedException{
        Servo waffleLeftS = hardwareMap.servo.get("waffleFrontS");
        Servo waffleRghtS = hardwareMap.servo.get("waffleBackS");
        waitForStart();
        Thread.sleep(5000);
        waffleLeftS.setPosition(0.35);
        waffleRghtS.setPosition(0.62);
        Thread.sleep(5000);
    }
}
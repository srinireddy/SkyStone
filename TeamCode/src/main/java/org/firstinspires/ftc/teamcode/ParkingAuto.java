package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
@Autonomous(name=" ",group="Sample")
public class ParkingAuto extends LinearOpMode{
    @Override public void runOpMode()throws InterruptedException{
        Servo waffleLeftS = hardwareMap.servo.get("succLeftS");
        Servo waffleRghtS = hardwareMap.servo.get("succRightS");
        waffleLeftS.setPosition(1.0);
        waffleRghtS.setPosition(0.0);
        waitForStart();
        Thread.sleep(2000);
        waffleLeftS.setPosition(0.4);
        waffleRghtS.setPosition(0.6);
        Thread.sleep(2000);
    }
}
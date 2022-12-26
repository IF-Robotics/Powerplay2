package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ConeStack")
public class ConeStack extends LinearOpMode {
    Servo leftArm,rightArm,frontArm,wrist,claw;
    DcMotor turret,slide;
    Rev2mDistanceSensor dist;
    @Override
    public void runOpMode() throws InterruptedException {
        leftArm = hardwareMap.get(Servo.class,"left");
        rightArm = hardwareMap.get(Servo.class,"right");
        frontArm = hardwareMap.get(Servo.class,"arm");
        wrist = hardwareMap.get(Servo.class,"wrist");
        claw = hardwareMap.get(Servo.class,"claw");
        turret = hardwareMap.get(DcMotor.class,"turret");
        slide = hardwareMap.get(DcMotor.class,"slide");
        dist = hardwareMap.get(Rev2mDistanceSensor.class,"dist");
    }
}

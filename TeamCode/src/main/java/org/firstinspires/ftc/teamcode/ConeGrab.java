package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Cone Grab")
public class ConeGrab extends LinearOpMode {
Servo leftArm,rightArm,frontArm,wrist,claw;
DcMotor turret;
    @Override
    public void runOpMode() throws InterruptedException {
        leftArm = hardwareMap.get(Servo.class,"left");
        rightArm = hardwareMap.get(Servo.class,"right");
        frontArm = hardwareMap.get(Servo.class,"arm");
        wrist = hardwareMap.get(Servo.class,"wrist");
        claw = hardwareMap.get(Servo.class,"claw");
        turret = hardwareMap.get(DcMotor.class,"turret");
        while(!isStopRequested()) {
            if (gamepad1.left_bumper) {
                claw.setPosition(0);
            }
            if (gamepad1.right_bumper) {
                claw.setPosition(1);
            }
            leftArm.setPosition(Math.abs(gamepad1.left_stick_y));
            rightArm.setPosition(1-Math.abs(gamepad1.left_stick_y));
            frontArm.setPosition(Math.abs(gamepad1.left_stick_y)+0.25);
            if (gamepad1.a)
                wrist.setPosition(0);
            if (gamepad1.b)
                wrist.setPosition(1);
            turret.setPower(gamepad1.right_stick_x);

        }
    }
}

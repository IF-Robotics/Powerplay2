package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Cone Grab")
public class ConeGrab extends LinearOpMode {
Servo leftArm,rightArm,frontArm,wrist,claw;
DcMotor turret,slide;
Rev2mDistanceSensor dist;
DigitalChannel magnet;
    @Override
    public void runOpMode() throws InterruptedException {
        int cones = 5;
        leftArm = hardwareMap.get(Servo.class,"left");
        rightArm = hardwareMap.get(Servo.class,"right");
        frontArm = hardwareMap.get(Servo.class,"arm");
        frontArm.setDirection(Servo.Direction.REVERSE);
        wrist = hardwareMap.get(Servo.class,"wrist");
        claw = hardwareMap.get(Servo.class,"claw");
        turret = hardwareMap.get(DcMotor.class,"turret");
        slide = hardwareMap.get(DcMotor.class,"slide");
        dist = hardwareMap.get(Rev2mDistanceSensor.class,"dist");
        magnet = hardwareMap.get(DigitalChannel.class,"magnet");
        magnet.setMode(DigitalChannel.Mode.INPUT);
        while(!isStopRequested()) {
            if (dist.getDistance(DistanceUnit.INCH)<1.5&&!gamepad1.left_bumper&&dist.getDistance(DistanceUnit.INCH)!=0.0) {
               // claw.setPosition(0.3);
            }
            else {
                //claw.setPosition(0.7);
            }
            if (!gamepad1.y) {
            leftArm.setPosition(Math.abs(gamepad1.left_stick_y)+0.2);
            rightArm.setPosition(0.8-Math.abs(gamepad1.left_stick_y));
            if (gamepad1.right_bumper)
                frontArm.setPosition(Math.abs(gamepad1.left_stick_y)+0.5);

            if (gamepad1.a)
                wrist.setPosition(0);
            if (gamepad1.b)
                wrist.setPosition(1);
            }
            //turret.setPower(gamepad1.right_stick_x);
            //slide.setPower(gamepad1.right_stick_y);
           // telemetry.addData("arm pos", leftArm.getPosition());
            //telemetry.addData("distance", dist.getDistance(DistanceUnit.INCH));
            //telemetry.addData("magnet",magnet.getState());
            //telemetry.addData("cones",cones);
            //TODO: Reverse Front Arm
            if (gamepad1.y) {
                wrist.setPosition(0);
                leftArm.setPosition(0.38+(cones-5)/20.0);
                rightArm.setPosition(1-(0.38+(cones-5)/20.0));
                frontArm.setPosition(0.68+(cones-5)/20.0);
                telemetry.addData("arm pos", leftArm.getPosition());
                telemetry.update();
                while (dist.getDistance(DistanceUnit.INCH)>1.5||dist.getDistance(DistanceUnit.INCH)==0.0) {
                    if (dist.getDistance(DistanceUnit.INCH)<1.5&&!gamepad1.left_bumper&&dist.getDistance(DistanceUnit.INCH)!=0.0) {
                        claw.setPosition(0.3);
                        slide.setPower(0);
                    }
                    else if (dist.getDistance(DistanceUnit.INCH)<10&&dist.getDistance(DistanceUnit.INCH)!=0) {
                        slide.setPower(0.4);
                    }
                    else {
                        slide.setPower(0.8);
                    }
                }
                claw.setPosition(0.3);
                slide.setPower(0);
                sleep(250);
                if (claw.getPosition()==0.3) {
                    leftArm.setPosition(0.5);
                    rightArm.setPosition(0.5);

                    sleep(250);
                    slide.setPower(-0.4);
                    wrist.setPosition(1);

                    while (magnet.getState())
                        slide.setPower(-0.8);
                    slide.setPower(0);
                    leftArm.setPosition(0.8);
                    rightArm.setPosition(0.2);
                    sleep(250);
                    claw.setPosition(0.6);
                    sleep(300);
                    leftArm.setPosition(0.5);
                    rightArm.setPosition(0.5);
                    sleep(100);
                    wrist.setPosition(0);

                    if (cones>0)
                        cones--;

                }
            }
            //telemetry.update();

        }
    }
}

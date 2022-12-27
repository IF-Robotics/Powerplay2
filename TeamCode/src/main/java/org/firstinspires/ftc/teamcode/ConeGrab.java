package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="Cone Grab")
public class ConeGrab extends LinearOpMode {
Servo leftArm,rightArm,frontArm,wrist,claw,tilt;
DcMotor turret,slide, rSlide,lSlide;
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
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide = hardwareMap.get(DcMotor.class,"slide");
        rSlide = hardwareMap.get(DcMotor.class,"upleft");
        rSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lSlide = hardwareMap.get(DcMotor.class,"upright");
        lSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tilt = hardwareMap.get(Servo.class,"slides");
        dist = hardwareMap.get(Rev2mDistanceSensor.class,"dist");
        magnet = hardwareMap.get(DigitalChannel.class,"magnet");
        magnet.setMode(DigitalChannel.Mode.INPUT);
        double tiltAmount = 0.5;

        WebcamName camName =  hardwareMap.get(WebcamName.class, "camera");
        OpenCvCamera cam = OpenCvCameraFactory.getInstance().createWebcam(camName);
        cam.openCameraDevice();
        PipelineKiwi pipeline = new PipelineKiwi();
        cam.setPipeline(pipeline);
        cam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
        pipeline.setColor(PipelineKiwi.DetectType.POLE);

        while(!isStopRequested()) {

            /*if (dist.getDistance(DistanceUnit.INCH)<1.5&&!gamepad1.left_bumper&&dist.getDistance(DistanceUnit.INCH)!=0.0) {
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
            }*/
            //turret.setPower(gamepad1.right_stick_x);
            //slide.setPower(gamepad1.right_stick_y);
           // telemetry.addData("arm pos", leftArm.getPosition());
            //telemetry.addData("distance", dist.getDistance(DistanceUnit.INCH));
            //telemetry.addData("magnet",magnet.getState());
            //telemetry.addData("cones",cones);
            tiltAmount+=gamepad1.left_stick_y/100;
            sleep(10);
            turret.setPower(gamepad1.left_stick_x/2);
            rSlide.setPower(gamepad1.right_stick_y);
            tilt.setPosition(tiltAmount);
            telemetry.addData("tilt",tiltAmount);
            telemetry.addData("turret",turret.getCurrentPosition());
            telemetry.addData("slide", rSlide.getCurrentPosition());
            telemetry.update();// pole is 27 in above axis of rotation
            if (gamepad1.a) {
                if (pipeline.getX()!=-404)
                    turret.setPower((pipeline.getX()-180)/360);
            }
            if (gamepad1.x) {
                turret.setTargetPosition(-365);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(1);
                rSlide.setTargetPosition(-2550);
                rSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rSlide.setPower(1);
                tilt.setPosition(0.27);
                while (turret.isBusy()|| rSlide.isBusy()) {

                }
                rSlide.setTargetPosition(5);
                rSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rSlide.setPower(1);
                while (turret.isBusy()|| rSlide.isBusy()) {

                }
                turret.setTargetPosition(0);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(1);
                tilt.setPosition(1);
                while (turret.isBusy()|| rSlide.isBusy()) {

                }
                turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (gamepad1.y) {
                wrist.setPosition(0);
                leftArm.setPosition(0.38+(cones-6)/20.0);
                rightArm.setPosition(1-(0.38+(cones-6)/20.0));
                frontArm.setPosition(0.68+(cones-6)/20.0);
                telemetry.addData("arm pos", leftArm.getPosition());
                telemetry.update();
                while (dist.getDistance(DistanceUnit.INCH)>1.5||dist.getDistance(DistanceUnit.INCH)==0.0) {
                    if (dist.getDistance(DistanceUnit.INCH)<1.5&&!gamepad1.left_bumper&&dist.getDistance(DistanceUnit.INCH)!=0.0) {
                        claw.setPosition(0.3);
                        slide.setPower(0);
                    }
                    else if (dist.getDistance(DistanceUnit.INCH)<10&&dist.getDistance(DistanceUnit.INCH)!=0) {
                        slide.setPower(0.25);
                    }
                    else {
                        slide.setPower(0.5);
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
                    claw.setPosition(0.54);
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

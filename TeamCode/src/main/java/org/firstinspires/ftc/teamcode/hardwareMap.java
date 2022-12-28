package org.firstinspires.ftc.teamcode;

import android.transition.Slide;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public abstract class hardwareMap extends LinearOpMode {
        public Servo leftArm, rightArm, frontArm, wrist, claw, tilt;
        public DcMotor turret, slide, rSlide, lSlide;
        public Rev2mDistanceSensor dist, bigboy;
        public DigitalChannel magnet;
        public Motor lf, rf, lb, rb;
    public void initizalize() {

        ElapsedTime elapsedTime = new ElapsedTime();
        int cones = 0;
        leftArm = hardwareMap.get(Servo.class, "left");
        rightArm = hardwareMap.get(Servo.class, "right");
        frontArm = hardwareMap.get(Servo.class, "arm");
        frontArm.setDirection(Servo.Direction.REVERSE);
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide = hardwareMap.get(DcMotor.class, "slide");
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rSlide = hardwareMap.get(DcMotor.class, "upleft");
        rSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lSlide = hardwareMap.get(DcMotor.class, "upright");
        lSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tilt = hardwareMap.get(Servo.class, "slides");
        dist = hardwareMap.get(Rev2mDistanceSensor.class, "dist");
        bigboy = hardwareMap.get(Rev2mDistanceSensor.class, "big boy");
        magnet = hardwareMap.get(DigitalChannel.class, "magnet");
        magnet.setMode(DigitalChannel.Mode.INPUT);
        lf = new Motor(hardwareMap, "lf");
        rf = new Motor(hardwareMap, "rf");
        lb = new Motor(hardwareMap, "lb");
        rb = new Motor(hardwareMap, "rb");
        MecanumDrive drive = new MecanumDrive(lf, rf, lb, rb);
        double tiltAmount = 1;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName camName = hardwareMap.get(WebcamName.class, "camera");
        OpenCvCamera cam = OpenCvCameraFactory.getInstance().createWebcam(camName, cameraMonitorViewId);
        cam.openCameraDevice();
        PipelineKiwi pipeline = new PipelineKiwi();
        cam.setPipeline(pipeline);
        cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        pipeline.setColor(PipelineKiwi.DetectType.POLE);
        int targetLift = 0;
        int turretPos = 0;
        double scoreTilt = 0;

        rightArm.setPosition(.9);
        frontArm.setPosition(.51);
        wrist.setPosition(0);
        claw.setPosition(.6);
        tilt.setPosition(1);
    }

}

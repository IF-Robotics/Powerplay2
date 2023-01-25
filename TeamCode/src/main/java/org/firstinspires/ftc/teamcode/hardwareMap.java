package org.firstinspires.ftc.teamcode;

import android.transition.Slide;

import java.lang.Math;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
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
        public Rev2mDistanceSensor dist/* bigboy*/;
        public DigitalChannel magnet;
        public DcMotor lf, rf, lb, rb;

    double strafeDriveTrainPower = 1;
    double forwardDriveTrainPower = .8;
    double Left_Stick_X = 0;
    double left_stick_y = 0;
    double Right_Stick_Y = 0;
    boolean reverse = false;
    int direction = 1;
    boolean home = true;
    ElapsedTime elapsedTime = new ElapsedTime();
    GamepadEx gamepadEx;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    boolean hasCone = false;

    boolean stuckMode = false;


    public void initizalize() {
        gamepadEx = new GamepadEx(gamepad1);

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
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rSlide = hardwareMap.get(DcMotor.class, "upleft");
        rSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lSlide = hardwareMap.get(DcMotor.class, "upright");
        lSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tilt = hardwareMap.get(Servo.class, "slides");
        dist = hardwareMap.get(Rev2mDistanceSensor.class, "dist");
    //    bigboy = hardwareMap.get(Rev2mDistanceSensor.class, "big boy");
        magnet = hardwareMap.get(DigitalChannel.class, "magnet");
        magnet.setMode(DigitalChannel.Mode.INPUT);
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

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


        rightArm.setPosition(.4);
        leftArm.setPosition(.4);
        frontArm.setPosition(.54);
        wrist.setPosition(0);
        claw.setPosition(.82);
        tilt.setPosition(1);
    }

    public void autoInit() {
        gamepadEx = new GamepadEx(gamepad1);

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
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rSlide = hardwareMap.get(DcMotor.class, "upleft");
        rSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lSlide = hardwareMap.get(DcMotor.class, "upright");
        lSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tilt = hardwareMap.get(Servo.class, "slides");
        dist = hardwareMap.get(Rev2mDistanceSensor.class, "dist");
      //  bigboy = hardwareMap.get(Rev2mDistanceSensor.class, "big boy");
        magnet = hardwareMap.get(DigitalChannel.class, "magnet");
        magnet.setMode(DigitalChannel.Mode.INPUT);
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        rightArm.setPosition(.4);
//        leftArm.setPosition(.4);
//        frontArm.setPosition(.54);
//        wrist.setPosition(0);
//        claw.setPosition(.82);
//        tilt.setPosition(1);
    }
}

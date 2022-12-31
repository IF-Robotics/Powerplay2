package org.firstinspires.ftc.teamcode;

import android.transition.Slide;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@TeleOp(name="Cone Grab")
@Disabled
public class ConeGrab extends LinearOpMode {
    Servo leftArm, rightArm, frontArm, wrist, claw, tilt;
    DcMotor turret, slide, rSlide, lSlide;
    Rev2mDistanceSensor dist, bigboy;
    DigitalChannel magnet;
    Motor lf, rf, lb, rb;
    ElapsedTime elapsedTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
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

        while (!isStopRequested()) {

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
            tiltAmount += gamepad1.left_stick_y / 100;
            sleep(10);

            rSlide.setPower(-gamepad1.right_stick_y);
            lSlide.setPower(gamepad1.right_stick_y);
            tilt.setPosition(tiltAmount);
            telemetry.addData("tilt", tiltAmount);
            telemetry.addData("turret", turret.getCurrentPosition());
            telemetry.addData("slide", rSlide.getCurrentPosition());
            // pole is 27 in above axis of rotation
            //2.35 in per 202 ticks
            //85.5 ticks/inch
            //0.3 servo per 20 degrees

            //66.67 degrees/servo
            // default servo is 15 degrees clockwise from 90
            if (gamepad1.a) {
                if (pipeline.getX() != -404) {
                    if (((pipeline.getX() - 320) / 360.0) > 0.1)
                        turret.setPower((pipeline.getX() - 320) / 360.0);
                    else
                        turret.setPower((0.1*(pipeline.getX() - 320) / 360.0)/Math.abs((pipeline.getX() - 320) / 360.0));
                }
                telemetry.addData("turret move ", (pipeline.getX() - 320) / 360.0);
            } else
                turret.setPower(gamepad1.left_stick_x / 2);

            telemetry.update();
            if (gamepad1.x) {
                double distance = bigboy.getDistance(DistanceUnit.INCH);
                int target = (int) (Math.sqrt(distance * distance + 27 * 27) * 70);
                rSlide.setTargetPosition(target);
                rSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lSlide.setTargetPosition(-target);
                lSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rSlide.setPower(1);
                lSlide.setPower(1);
                telemetry.addData("target", target);

                tilt.setPosition((75-Math.toDegrees(Math.atan(27 / distance))) / 66.67 + 0.5);
                telemetry.addData("distance",bigboy.getDistance(DistanceUnit.INCH));
                telemetry.addData("angle",(Math.toDegrees(Math.atan(27 / distance))));
                telemetry.addData("tilt pos", tilt.getPosition());
                telemetry.update();
                while (turret.isBusy() || rSlide.isBusy()) {

                }
                sleep(1000);
                rSlide.setTargetPosition(-5);
                rSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rSlide.setPower(1);
                lSlide.setTargetPosition(5);
                lSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lSlide.setPower(1);
                while (turret.isBusy() || rSlide.isBusy()) {

                }
                turret.setTargetPosition(0);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(1);
                tilt.setPosition(1);
                while (turret.isBusy() || rSlide.isBusy()) {

                }
                turret.setPower(0);
                rSlide.setPower(0);
                lSlide.setPower(0);
                turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            frontArm.setPosition(0.3);
            if (gamepad1.right_bumper) {
                scoreTilt = tilt.getPosition();
                turretPos = turret.getCurrentPosition();
                targetLift = Range.clip(rSlide.getCurrentPosition(),-2500,2500);
            }
            if (gamepad1.left_bumper) {
                turret.setTargetPosition(turretPos);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rSlide.setTargetPosition(targetLift);
                rSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lSlide.setTargetPosition(-targetLift);
                lSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rSlide.setPower(1);
                lSlide.setPower(1);
                turret.setPower(1);
                telemetry.addData("target", targetLift);

                tilt.setPosition(scoreTilt);
                telemetry.addData("tilt pos", tilt.getPosition());
                telemetry.update();
                while (!compare(lSlide.getCurrentPosition(),lSlide.getTargetPosition(),25)) {

                }
                rSlide.setPower(0);
                lSlide.setPower(0);
                //sleep(100);
                rSlide.setTargetPosition(0);
                rSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rSlide.setPower(1);
                lSlide.setTargetPosition(0);
                lSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lSlide.setPower(1);
                tilt.setPosition(0.2);

                while (lSlide.isBusy() || rSlide.isBusy()) {

                }
                rSlide.setPower(0);
                lSlide.setPower(0);
                rSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                turret.setTargetPosition(0);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(1);

                tiltAmount = 1;
                while (turret.isBusy()) {

                }
                turret.setPower(0);
                sleep(10);
                turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }


            claw.setPosition(0.55);
            if (gamepad1.y) {
                wrist.setPosition(0);
                leftArm.setPosition(0.38 + (cones - 5) / 20.0);
                rightArm.setPosition(1 - (0.38 + (cones - 5) / 20.0));
                frontArm.setPosition(0.68 + (cones - 4) / 20.0);
                telemetry.addData("arm pos", leftArm.getPosition());
                telemetry.update();
                while (dist.getDistance(DistanceUnit.INCH) > 1.5 || dist.getDistance(DistanceUnit.INCH) == 0.0) {
                    if (dist.getDistance(DistanceUnit.INCH) < 1.5 && !gamepad1.left_bumper && dist.getDistance(DistanceUnit.INCH) != 0.0) {
                        claw.setPosition(0.3);
                        slide.setPower(0);
                    } else if (dist.getDistance(DistanceUnit.INCH) < 10 && dist.getDistance(DistanceUnit.INCH) != 0) {
                        slide.setPower(0.25);
                    } else {
                        slide.setPower(0.7);
                    }
                }
                claw.setPosition(0.3);
                slide.setPower(0);
                sleep(250);
                if (dist.getDistance(DistanceUnit.INCH) < 1.5 && !gamepad1.left_bumper && dist.getDistance(DistanceUnit.INCH) != 0.0) {
                    leftArm.setPosition(0.5);
                    rightArm.setPosition(0.5);

                    sleep(250);
                    slide.setPower(-0.4);
                    wrist.setPosition(1);

                    while (magnet.getState())
                        slide.setPower(-0.8);
                    elapsedTime.reset();
                    slide.setPower(-0.4);
                    while (magnet.getState()||elapsedTime.milliseconds()<200)
                        slide.setPower(-0.4);
                    slide.setPower(0);
                    leftArm.setPosition(0.8);
                    rightArm.setPosition(0.2);
                    elapsedTime.reset();
                    slide.setPower(-0.4);
                    while (magnet.getState()||elapsedTime.milliseconds()<200)
                        slide.setPower(-0.4);
                    slide.setPower(0);
                    claw.setPosition(0.6);

                    sleep(300);
                    leftArm.setPosition(0.5);
                    rightArm.setPosition(0.5);
                    sleep(100);
                    wrist.setPosition(0);

                    if (cones > 0)
                        cones--;

                }
            }
            //telemetry.update();
            drive.driveRobotCentric(-gamepad2.left_stick_x, gamepad2.left_stick_y, -gamepad2.right_stick_x, true);
        }
    }
    public boolean compare(int value,int compareTo, int difference) {
        return Math.abs(value - compareTo)<difference;
    }
}
package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

//@Config
@Autonomous(name="⬅️1➕5 \uD83D\uDDFC")
public class RightAuto extends hardwareMap{

    static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections
            = RevHubOrientationOnRobot.LogoFacingDirection.values();
    static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections
            = RevHubOrientationOnRobot.UsbFacingDirection.values();
    static int LAST_DIRECTION = logoFacingDirections.length - 1;
    static float TRIGGER_THRESHOLD = 0.2f;


    FtcDashboard dashboard;
    public static double error = 5.1;
    public static double p = .02;
    public static double botHeading;
    public static double reference = botHeading;


    IMU imu;
    int logoFacingDirectionPosition;
    int usbFacingDirectionPosition;
    boolean orientationIsValid = true;
    YawPitchRollAngles orientation;

    double power = 1;
    double armPosition = .65;
    double frontArmPosition = .65;
    double tiltPosition = .3;
    int turretPosition = -660;
    int elevatePosition = 700;

    OpenCvCamera camera;
    AprilTagPipeline aprilTagPipeline;
    static final double FEET_PER_METER = 3.28084;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.05;
    int position = 0;
    ElapsedTime timer = new ElapsedTime();
    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {
        initizalize();
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        //IMU
        imu = hardwareMap.get(IMU.class, "imu");
        logoFacingDirectionPosition = 5;
        usbFacingDirectionPosition = 0;
        updateOrientation();
        orientation = imu.getRobotYawPitchRollAngles();

        boolean justChangedLogoDirection = false;
        boolean justChangedUsbDirection = false;


        botHeading = -orientation.getYaw(AngleUnit.DEGREES);

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        timer.reset();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagPipeline = new AprilTagPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
            }
        });

// Now use these simple methods to extract each angle
// (Java type double) from the object you just created:
        cameraWaitForStart();
        resetEncoders();
        imu.resetYaw();
        sleep(100);
//tilt the tilt
        tilt.setPosition(.3);
//drive forward
        forward();
//move turret
        turret.setTargetPosition(turretPosition);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1);
//turn
        turn();
//score preload
        preload();
//grab the cone
//armposition - .9      frontArmPosition - .3

        for(int i=0; i<5; i++) {
            cycle(armPosition, frontArmPosition, turretPosition, elevatePosition, tiltPosition);
            armPosition += (Math.abs(.9 - armPosition))/5;
            frontArmPosition -= (Math.abs(.3 - frontArmPosition))/5;
        }
//go to park
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0);
        park();
    }

    void updateOrientation() {
        RevHubOrientationOnRobot.LogoFacingDirection logo = logoFacingDirections[logoFacingDirectionPosition];
        RevHubOrientationOnRobot.UsbFacingDirection usb = usbFacingDirections[usbFacingDirectionPosition];
        try {
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            orientationIsValid = true;
        } catch (IllegalArgumentException e) {
            orientationIsValid = false;
        }
    }
    public void elevator(int elevatePosition, double elevatePower){
        lSlide.setTargetPosition(elevatePosition);
        rSlide.setTargetPosition(elevatePosition);
        lSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lSlide.setPower(elevatePower);
        rSlide.setPower(elevatePower);
    }
    public void preload() {
        sleep(300);
        //extend out the slides
        slide.setTargetPosition(-440);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(1);
        //move the arm
        rightArm.setPosition(armPosition);
        leftArm.setPosition(armPosition);
        claw.setPosition(.56);
        frontArm.setPosition(frontArmPosition);
        wrist.setPosition(.14);
        //raise the elevator
        elevator(700, 1);
        //move the elevator and turret
        while (lSlide.getCurrentPosition() < (lSlide.getTargetPosition() - 5)) {
            sleep(1);
        }
        sleep(100);
        elevator(2, 1);
        sleep(300);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1);
        tilt.setPosition(.46);
    }
    public void cycle(double armPosition, double frontArmPosition, int turretPosition, int elevatorPosition, double tiltPosition) {
        //move out the slides
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setPower(-.25);
        while ((dist.getDistance(DistanceUnit.INCH) > 1 && dist.getDistance(DistanceUnit.INCH) > 0)) {
            sleep(1);
        }
        slide.setPower(0);
        //pick up the cone
        claw.setPosition(.82);
        slide.setPower(0);
        sleep(150);
        rightArm.setPosition(.4);
        leftArm.setPosition(.4);
        sleep(300);
//second part
        //move the slides back
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(1);
        //bring the arm up
        rightArm.setPosition(.4);
        leftArm.setPosition(.4);
        wrist.setPosition(.81);
        frontArm.setPosition(.54);
        claw.setPosition(.82);
        sleep(400);
        //close claw
        claw.setPosition(.82);
        elevator(13, 1);
        //move arm
        rightArm.setPosition(0.25);
        leftArm.setPosition(.25);
        wrist.setPosition(.81);
        sleep(200);
        //open claw
        claw.setPosition(.56);
        sleep(200);
        //move arm out of the way
        rightArm.setPosition(.4);
        leftArm.setPosition(.4);
        sleep(100);
        wrist.setPosition(.81);
        frontArm.setPosition(.54);
        claw.setPosition(.82);
        sleep(100);
        //move the turret
        tilt.setPosition(tiltPosition);
        turret.setTargetPosition(turretPosition);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1);
        sleep(200);
        rightArm.setPosition(armPosition);
        leftArm.setPosition(armPosition);
        frontArm.setPosition(frontArmPosition);
        wrist.setPosition(.14);
        claw.setPosition(.56);
        slide.setTargetPosition(-440);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(1);
        //raise the elevator
        elevator(elevatorPosition, 1);
        //move the elevator and turret
        while (lSlide.getCurrentPosition() < (lSlide.getTargetPosition() - 5)) {
            sleep(1);
        }
        sleep(100);
        elevator(2, 1);
        sleep(300);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1);
        tilt.setPosition(.46);
    }
    public void turn() {
        power = 1;
        while (opModeIsActive() && botHeading > -90) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            // obtain the encoder position
            botHeading = -orientation.getYaw(AngleUnit.DEGREES);
            // calculate the error
            error = reference - botHeading;

            if(botHeading < -65) {
                power = .4;
            }
            //turn
            rf.setPower(power * -.5);
            rb.setPower(power * -.5);
            lf.setPower(power * -.5);
            lb.setPower(power * -.5);

            telemetry.addData("pose", botHeading);
            telemetry.addData("error", error);
            telemetry.addData("logo Direction (set with bumpers)", logoFacingDirections[logoFacingDirectionPosition]);
            telemetry.addData("usb Direction (set with triggers)", usbFacingDirections[usbFacingDirectionPosition] + "\n");
            telemetry.addData("lf", lf.getPower());
            telemetry.addData("rf", rf.getPower());
            telemetry.addData("target", reference);
            telemetry.update();
        }
        rf.setPower(0);
        rb.setPower(0);
        lf.setPower(0);
        lb.setPower(0);
    }
    public void forward() {
        while (opModeIsActive() && lb.getCurrentPosition() > -94000) {

            if (lb.getCurrentPosition() < -50000) {
                power = .17;
            }
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            // obtain the encoder position
            botHeading = -orientation.getYaw(AngleUnit.DEGREES);
            // calculate the error
            error = reference - botHeading;


                // set motor power proportional to the error
                if (error >= 0) {
                    lf.setPower(power*(.8 + error * p));
                    lb.setPower(power*(.8 + error * p));
                    rb.setPower(-power*(.8));
                    rf.setPower(-power*(.8));
                } else {
                    lf.setPower(power*(.8));
                    lb.setPower(power*(.8));
                    rb.setPower(-power*(.8 - error * p));
                    rf.setPower(-power*(.8 - error * p));
                }
                telemetry.addData("pose", botHeading);
                telemetry.addData("error", error);
                telemetry.addData("logo Direction (set with bumpers)", logoFacingDirections[logoFacingDirectionPosition]);
                telemetry.addData("usb Direction (set with triggers)", usbFacingDirections[usbFacingDirectionPosition] + "\n");
                telemetry.addData("lf", lf.getPower());
                telemetry.addData("rf", rf.getPower());
                telemetry.addData("target", reference);
                telemetry.update();

        }
    }
    public void park() {
        //TODO: Micah, fill in these values
        if(position == 1) {
            //forward
        } else if(position == 2) {
            //stay in same tile
            rf.setPower(0);
            rb.setPower(0);
            lf.setPower(0);
            lb.setPower(0);
            sleep(1000);
        } else {
            //backwards into the wall
        }
    }
    public void resetEncoders() {
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    public void cameraWaitForStart() {
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagPipeline.getLatestDetections();
            if (currentDetections.size() != 0) {
                boolean tagFound = false;
                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id < 6 || tag.id > 2) {
                        timer.reset();
                        tagOfInterest = tag;
                        tagFound = true;
                        if (tag.id == 4) {
                            position = 1;
                        } else if (tag.id == 5) {
                            position = 2;
                        } else { //if tag=3
                            position = 3;
                        }
                        break;
                    }
                }
                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    telemetry.addData("Position", position);
                    telemetry.addData("Timer", timer.milliseconds());
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");
                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }
            } else {
                telemetry.addLine("Don't see tag of interest :(");
                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else if (timer.milliseconds() < 1000) {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("too long");
                    position = 0;
                    telemetry.addData("position", position);
                }
            }
        }
    }
}
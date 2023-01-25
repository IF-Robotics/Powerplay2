package org.firstinspires.ftc.teamcode;
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
    double armPosition = .66;
    double frontArmPosition = .65;
    double tiltPosition = .3;
    int turretPosition = -700;
    int elevatePosition = 700;

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
// Now use these simple methods to extract each angle
// (Java type double) from the object you just created:
        waitForStart();
        resetEncoders();
        imu.resetYaw();
        sleep(200);
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
//armposition - .9      frontArmPosition - .35

        for(int i=0; i<=5; i++) {
            cycle(armPosition, frontArmPosition, turretPosition, elevatePosition, tiltPosition);
            armPosition -= (Math.abs(.9 - armPosition))/5;
            frontArmPosition += (Math.abs(.35 - frontArmPosition))/5;
        }
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
    }
    public void cycle(double armPosition, double frontArmPosition, int turretPosition, int elevatorPosition, double tiltPosition) {
        //move out the slides
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setPower(-.5);
        while ((dist.getDistance(DistanceUnit.INCH) > 1 && dist.getDistance(DistanceUnit.INCH) > 0)) {
            sleep(1);
        }
        slide.setPower(0);
        //pick up the cone
        claw.setPosition(.82);
        slide.setPower(-.01);
        sleep(200);
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
        sleep(500);
        tilt.setPosition(.3);
        //close claw
        claw.setPosition(.82);
        sleep(100);
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

            if (Math.abs(error) > 0) {

                // set motor power proportional to the error
                if (error > 0) {
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
}
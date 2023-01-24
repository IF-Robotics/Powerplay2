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
@Autonomous(name="RightAuto")
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

        double power = 1;

        botHeading = -orientation.getYaw(AngleUnit.DEGREES);

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
// Now use these simple methods to extract each angle
// (Java type double) from the object you just created:
        waitForStart();

        imu.resetYaw();
//tilt the tilt
        tilt.setPosition(.3);
//drive forward
        while (opModeIsActive() && lb.getCurrentPosition() > -94000) {

            if (lb.getCurrentPosition() < -54000) {
                power = .15;
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
//move turret
        turret.setTargetPosition(-720);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1);
//turn
        power = 1;
        while (opModeIsActive() && botHeading > -90) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            // obtain the encoder position
            botHeading = -orientation.getYaw(AngleUnit.DEGREES);
            // calculate the error
            error = reference - botHeading;

            if(botHeading < -60) {
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
//score preload
        sleep(300);
        //extend out the slides
        slide.setTargetPosition(-500);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(1);
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
        sleep(1000);
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
}
package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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

@Config
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

        double power = 0;

        botHeading = -orientation.getYaw(AngleUnit.DEGREES);

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
// Now use these simple methods to extract each angle
// (Java type double) from the object you just created:
        waitForStart();


        while (opModeIsActive()) {
            if (gamepad1.touchpad) {
                imu.resetYaw();
                reference = botHeading;
                sleep(1000);
                power = 1;

            }
            if (gamepad1.dpad_up){
                power = 0;
            }

            if (Math.abs(error) < 0) {
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                // obtain the encoder position
                botHeading = -orientation.getYaw(AngleUnit.DEGREES);
                // calculate the error
                error = reference - botHeading;
                // set motor power proportional to the error
                if (error > 0) {
                    lf.setPower(power*(.5 + error * p));
                    lb.setPower(power*(.5 + error * p));
                    rb.setPower(-power*(.5));
                    rf.setPower(-power*(.5));
                } else {
                    lf.setPower(power*(.5));
                    lb.setPower(power*(.5));
                    rb.setPower(-power*(.5 + error * p));
                    rf.setPower(-power*(.5 + error * p));
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
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Autonomous(name="RightAuto")
public class RightAuto extends hardwareMap{
    @Override
    public void runOpMode() throws InterruptedException {
        //IMU
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
        initizalize();
        double error = 5.1;
        double botHeading = -imu.getAngularOrientation().firstAngle;
        waitForStart();
        double reference = botHeading;

        while (Math.abs(error) > 5) {
            // obtain the encoder position
            botHeading = -imu.getAngularOrientation().firstAngle;
            // calculate the error
            error = reference - botHeading;
            // set motor power proportional to the error
           // armMotor.setPower(error);
            telemetry.addData("pose", botHeading);

        }
    }

}
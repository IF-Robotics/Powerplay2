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
@TeleOp(name="servoTests")

public class servoTests extends hardwareMap{

    @Override
    public void runOpMode() throws InterruptedException {
        init();
        waitForStart();
        while (opModeIsActive()) {
            if (dist.getDistance(DistanceUnit.INCH) < 1.5 && !gamepad1.left_bumper && dist.getDistance(DistanceUnit.INCH) != 0.0) {
                // claw.setPosition(0.3);
            } else {
                //claw.setPosition(0.7);
            }
            if (!gamepad1.y) {
                leftArm.setPosition(Math.abs(gamepad1.left_stick_y) + 0.2);
                rightArm.setPosition(0.8 - Math.abs(gamepad1.left_stick_y));
                if (gamepad1.right_bumper)
                    frontArm.setPosition(Math.abs(gamepad1.left_stick_y) + 0.5);

                if (gamepad1.a)
                    wrist.setPosition(0);
                if (gamepad1.b)
                    wrist.setPosition(1);
            }
            turret.setPower(gamepad1.right_stick_x);
            slide.setPower(gamepad1.right_stick_y);
            telemetry.addData("arm pos", leftArm.getPosition());
            telemetry.addData("distance", dist.getDistance(DistanceUnit.INCH));
            telemetry.addData("magnet", magnet.getState());
            telemetry.update();
        }
    }
}

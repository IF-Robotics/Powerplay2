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
@TeleOp(name="lol")
public class lol extends hardwareMap{

    @Override
    public void runOpMode() throws InterruptedException {
        initizalize();
        waitForStart();
        int cones = 0;
        ElapsedTime elapsedTime = new ElapsedTime();
        while (opModeIsActive()) {
            double distance = bigboy.getDistance(DistanceUnit.INCH);
            if (gamepad1.right_bumper || distance < .5 && distance > 0){

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
            } else {

            }
        }

    }
}

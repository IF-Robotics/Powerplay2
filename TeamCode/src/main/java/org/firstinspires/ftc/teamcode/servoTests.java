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
import java.util.Arrays;
@TeleOp(name="servoTests")

public class servoTests extends hardwareMap{

    int x = 0;
Servo[] servo = {lift, tilt, rightArm, wrist,frontArm , claw};
double[] servoPosition = {.5, .5, .5, .5, .5, .5, .5};
String servoName;
    @Override
    public void runOpMode() throws InterruptedException {
        servo[2] = hardwareMap.get(Servo.class, "right");
        servo[1] = hardwareMap.get(Servo.class, "arm");
        servo[1].setDirection(Servo.Direction.REVERSE);
        servo[3] = hardwareMap.get(Servo.class, "wrist");
        servo[4] = hardwareMap.get(Servo.class, "claw");
        servo[0] = hardwareMap.get(Servo.class, "lift");
        initizalize();
        waitForStart();
        while (opModeIsActive()) {
            try {

                previousGamepad1.copy(currentGamepad1);
                previousGamepad2.copy(currentGamepad2);

                currentGamepad1.copy(gamepad1);
                currentGamepad2.copy(gamepad2);
            }
            catch (Exception e) {

            }

            if (currentGamepad1.touchpad && !previousGamepad1.touchpad) {
                x++;
            }
            if (x >= servo.length - 1) {
               x = 0;
            }

            if (gamepad1.dpad_right) {
                servoPosition[x]+=.01;
            }
            if (gamepad1.dpad_left) {
                servoPosition[x]-=.01;
            }
            sleep(100);
            servo[x].setPosition(servoPosition[x]);

            if(x==0){
                servoName = "claw";
            }
            if(x==1){
                servoName = "frontArm";
            }
            if(x==2){
                servoName = "rightArm";
            }
            if(x==3){
                servoName = "wrist";
            }
            if(x==4){
                servoName = "tilt";
            }
            if(x==5){
                servoName = "lift";
            }
            telemetry.addData("servo", servoName);
            telemetry.addData("ServoPosition", servo[x].getPosition());
            telemetry.addData("turret", turret.getCurrentPosition());
            telemetry.addData("slides", slide.getCurrentPosition());
            System.out.print(Arrays.toString(servo));
       //     telemetry.addData("elevator", )
            telemetry.update();

        }
    }
}

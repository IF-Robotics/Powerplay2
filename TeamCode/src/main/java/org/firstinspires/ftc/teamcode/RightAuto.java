package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Autonomous(name="lol")
public class RightAuto extends hardwareMap{
    @Override
    public void runOpMode() throws InterruptedException {
        initizalize();
        waitForStart();
        double reference = someValue;
        while (Math.abs(error) > tolerance) {
            // obtain the encoder position
            encoderPosition = armMotor.getPosition();
            // calculate the error
            error = reference - encoderPosition;
            // set motor power proportional to the error
            armMotor.setPower(error);
        }
    }

}
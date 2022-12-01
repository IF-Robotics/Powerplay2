package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="servo")
@Disabled
public class servoTest extends Hardwaremap{
    @Override
    public void runOpMode() throws InterruptedException {
        teleopInit();
        waitForStart();
        while (!isStopRequested()) {
            wrist.setPosition(gamepad1.left_stick_y);
            wrist2.setPosition(1-gamepad1.left_stick_y);
            telemetry.addData("left y", gamepad1.left_stick_y);
            //0.7 is good number
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class BasicAuto extends CameraShortcut {

    @Override
    public void runOpMode(){
        //close the claw
        claw.setPosition(.59);
        initCamera();
        waitForStart();
        signalPosition position = getSignalPosition();
        telemetry.addData("Position", position);
        telemetry.update();

        switch(position) {
            case One:
                zoneOnePath();
                break;
            case Two:
                zoneTwoPath();
                break;
            case Three:
                zoneThreePath();
                break;
            default:
                zoneOnePath();
        }
    }

    public void zoneThreePath() {
        strafe(.5, 1, 1400);
        drive(.3, -1, 600);
        drive(.5, 1, 1000);
    }
    public void zoneTwoPath() {
        
        drive(.5, 1, 1000);
    }
    public void zoneOnePath() {
        strafe(.5, -1, 1300);
        drive(.3, -1, 600);
        drive(.5, 1, 1000);
    }

    public void strafe(double power, int direction, int time) {
        front_Left.setPower(power*direction);
        back_Leftx.setPower(-1*power*direction);
        back_Right.setPower(-1*power*direction);
        front_Right.setPower(power*direction);
        sleep(time);
        front_Left.setPower(0);
        back_Leftx.setPower(0);
        back_Right.setPower(0);
        front_Right.setPower(0);
        sleep(100);
    }
    public void drive(double power, int direction, int time) {
        front_Left.setPower(power*direction);
        back_Leftx.setPower(power*direction);
        back_Right.setPower(-1*power*direction);
        front_Right.setPower(-1*power*direction);
        sleep(time);
        front_Left.setPower(0);
        back_Leftx.setPower(0);
        back_Right.setPower(0);
        front_Right.setPower(0);
        sleep(100);
    }
}

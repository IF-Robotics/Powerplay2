package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class BasicAuto extends CameraShortcut {

    @Override
    public void runOpMode() {
        initCamera();
        autoInit();
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
        strafe(.5, -1, 1500);
        drive(.3, -1, 1000);
        drive(.5, 1, 1500);
    }
    public void zoneTwoPath() {
        
        drive(.5, 1, 1500);
    }
    public void zoneOnePath() {
        strafe(.5, 1, 1500);
        drive(.3, -1, 1000);
        drive(.5, 1, 1500);
    }
}

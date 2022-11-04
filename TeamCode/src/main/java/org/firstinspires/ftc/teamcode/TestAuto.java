package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestAuto extends CameraShortcut {
    //private CameraShortcut camera = new CameraShortcut();

    @Override
    public void runOpMode(){
        initCamera();
        waitForStart();
        signalPosition position = getSignalPosition();
        telemetry.addData("Position", position);
    }
}

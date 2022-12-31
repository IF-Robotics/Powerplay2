package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="CVTEST")
@Disabled
public class CamTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        WebcamName camName =  hardwareMap.get(WebcamName.class, "camera");
        OpenCvCamera cam = OpenCvCameraFactory.getInstance().createWebcam(camName);
        cam.openCameraDevice();
        PipelineKiwi pipeline = new PipelineKiwi();
        cam.setPipeline(pipeline);
        cam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
        pipeline.setColor(PipelineKiwi.DetectType.POLE);

        while (!isStopRequested()) {
            telemetry.addData("x",pipeline.getX());
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="a cacti cousin",group="A cacti Cousin")
public class OpenCVFTW extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY);
        WebcamName LName = hardwareMap.get(WebcamName.class, "lcam");
        OpenCvCamera lcam = OpenCvCameraFactory.getInstance().createWebcam(LName,viewportContainerIds[0]);
        lcam.openCameraDevice();
        PoleDetect lPipe = new PoleDetect();
        lcam.setPipeline(lPipe);



        WebcamName RName = hardwareMap.get(WebcamName.class, "rcam");
        OpenCvCamera rcam = OpenCvCameraFactory.getInstance().createWebcam(RName,viewportContainerIds[1]);
        rcam.openCameraDevice();
        PoleDetect rPipe = new PoleDetect();
        rcam.setPipeline(rPipe);
        sleep(500);

        startCameras(lcam,rcam);

        while(!isStopRequested()) {
            telemetry.addData("left x", lPipe.getX());
            telemetry.addData("left y", lPipe.getY());
            telemetry.addData("right x",rPipe.getX());
            telemetry.addData("right y",rPipe.getY());
            telemetry.update();

        }
    }
    public void startCameras(OpenCvCamera lcam, OpenCvCamera rcam) {
        if (!isStopRequested()) {
            try {
                lcam.openCameraDevice();
                rcam.openCameraDevice();
                lcam.startStreaming(640,480,OpenCvCameraRotation.UPRIGHT);
                rcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }
            catch (Exception e) {
                telemetry.addData("error",e);
                telemetry.update();
                startCameras(lcam,rcam);
            }
        }
    }
}

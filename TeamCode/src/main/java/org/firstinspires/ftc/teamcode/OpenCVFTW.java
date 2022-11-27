package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PoleDetect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="a cacti cousin",group="A cacti Cousin")
public class OpenCVFTW extends Hardwaremap {
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
        teleopInit();
        while(!isStopRequested()) {
            telemetry.addData("left x", lPipe.getX());
            telemetry.addData("left y", lPipe.getY());
            telemetry.addData("right x",rPipe.getX());
            telemetry.addData("right y",rPipe.getY());

            int lDist = lPipe.getX();
            int rDist = 640 - rPipe.getX();
            double xPower = 0;
            double spin = 0;
            if (gamepad1.a) {
                telemetry.addData("gamepad1",'a');
                if (lDist!=-404&&rDist!=1044) {
                    telemetry.addData("doenst see pole", true);
                    if (Math.abs(rDist-lDist)>100) {
                        spin = (rDist-lDist)/1300.0;

                        telemetry.addData("spin", spin);
                    }
                    if (rDist+lDist<1800) {
                        xPower = -(rDist+lDist-400)/1200.0;
                        xPower = Range.clip(xPower,-0.3,0.3);
                        telemetry.addData("xPower", xPower);
                    }
                    move(xPower,spin);
                }
            }
            else
                stopMotors();
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
    public void move(double forward, double spin) {
        front_Left.setPower(forward+spin);
        back_Leftx.setPower(forward+spin);
        front_Right.setPower(forward-spin);
        back_Right.setPower(forward-spin);
    }
    public void stopMotors() {
        front_Left.setPower(0);
        back_Leftx.setPower(0);
        front_Right.setPower(0);
        back_Right.setPower(0);
    }
}

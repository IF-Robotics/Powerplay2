package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Autonomous
public class AprilTagTest extends hardwareMap
{
    OpenCvCamera camera;
    AprilTagPipeline aprilTagPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.05;

    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    int position = 0;
    ElapsedTime timer = new ElapsedTime();

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        timer.reset();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagPipeline = new AprilTagPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);
        autoInit();

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id <6 || tag.id >2)
                    {
                        timer.reset();
                        tagOfInterest = tag;
                        tagFound = true;
                        if(tag.id == 4) {
                            position = 1;
                        } else if(tag.id == 5) {
                            position = 2;
                        } else { //if tag=3
                            position = 3;
                        }
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    telemetry.addData("Position", position);
                    telemetry.addData("Timer", timer.milliseconds());
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else if (timer.milliseconds() < 1000)
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("too long");
                    position = 0;
                    telemetry.addData("position", position);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null)
        {
            /*
             * Insert your autonomous code here, presumably running some default configuration
             * since the tag was never sighted during INIT
             */
        }
        else
        {
            /*
             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
             */

            // e.g.
            if(tagOfInterest.pose.x <= 20)
            {
                // do something
            }
            else if(tagOfInterest.pose.x >= 20 && tagOfInterest.pose.x <= 50)
            {
                // do something else
            }
            else if(tagOfInterest.pose.x >= 50)
            {
                // do something else
            }
        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        //while (opModeIsActive()) {
            if(position == 1) {
                //left
                strafe(.8, 1, 750);
                drive(.8, -1, 300);
                drive(.8, 1, 1000);
            } else if (position == 2) {
                strafe(.8, -1, 250);
                drive(.5, 1, 1600);
                drive(.5, -1, 600);
            } else {
                strafe(.8, -1, 900);
                drive(.8, -1, 300);
                drive(.8, 1, 1000);
            }
        //}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    public void strafe(double power, int direction, int time) {
        lf.setPower(power*direction);
        lb.setPower(-1*power*direction);
        rb.setPower(power*direction);
        rf.setPower(-1*power*direction);
        sleep(time);
        lf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
        rf.setPower(0);
        sleep(100);
    }
    public void drive(double power, int direction, int time) {
        lf.setPower(-power*direction);
        lb.setPower(-power*direction);
        rb.setPower(-power*direction);
        rf.setPower(-power*direction);
        sleep(time);
        lf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
        rf.setPower(0);
        sleep(100);
    }
}

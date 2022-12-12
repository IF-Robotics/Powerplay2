package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SignalDetect extends OpenCvPipeline {
    double x=100;
    double y=120;
    boolean viewportPaused;
    Mat dst = new Mat();
    Mat hsv = new Mat();
    Mat gtresh = new Mat();
    Mat ytresh = new Mat();
    Mat ghierarchy = new Mat();
    Mat yhierarchy = new Mat();
    Mat inputResized = new Mat();
    Scalar color = new Scalar(0,255,0);
    Telemetry telemetry;
    double maxArea = 0;
    MatOfPoint maxContour;
    boolean maxType=false;
    int pos=-1;

    @Override
    public Mat processFrame(Mat input) {


        List<MatOfPoint> gcontours = new ArrayList<>();
        List<MatOfPoint> ycontours = new ArrayList<>();
        Imgproc.resize(input,inputResized, new Size(320,240));
        Imgproc.GaussianBlur(inputResized, dst, new Size(5,5), 0);
        Imgproc.cvtColor(dst, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, new Scalar(40,40,40), new Scalar(70,255,255), gtresh);
        Core.inRange(hsv, new Scalar(20,120,120), new Scalar(30,255,255), ytresh);

        Imgproc.findContours(gtresh, gcontours, ghierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
        Imgproc.findContours(ytresh, ycontours, yhierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
        maxArea=0;
        maxContour=null;
        pos=-1;
        for(int i = 0; i < ycontours.size(); i++) {
            double area = Imgproc.contourArea(ycontours.get(i));
            if (area> maxArea&&area>300) {
                MatOfPoint point2 = ycontours.get(i);
                double[] position = point2.get(0,0);

                maxArea=area;
                if (position[0]<120)
                    maxContour=ycontours.get(i);
                maxType=true;
            }
            if(area>300){
                Imgproc.drawContours(inputResized, ycontours, -1, new Scalar(200,20,2), 2
                        , Imgproc.LINE_8, yhierarchy, 2, new Point());
                MatOfPoint point2 = ycontours.get(i);
                double[] position = point2.get(0,0);
                int x = (int) position[0];
                pos = x;
            }
        }
        maxArea = 0;
        for(int i = 0; i < gcontours.size(); i++) {
            double area = Imgproc.contourArea(gcontours.get(i));
            if (area> maxArea&&area>300) {
                MatOfPoint point2 = gcontours.get(i);
                double[] position = point2.get(0,0);

                maxArea=area;
                if (position[0]<120)
                    maxContour=gcontours.get(i);
                maxType=false;
            }
            if(area>300){
                Imgproc.drawContours(inputResized, gcontours, -1, color, 2
                        , Imgproc.LINE_8, ghierarchy, 2, new Point());
                MatOfPoint point2 = gcontours.get(i);
                double[] position = point2.get(0,0);
                int x = (int) position[0];
            }
        }

        if (maxContour !=null) {
            MatOfPoint2f thing = new MatOfPoint2f(maxContour.toArray());
            RotatedRect rect = Imgproc.minAreaRect(thing);
            MatOfPoint2f approx = new MatOfPoint2f();
            Imgproc.approxPolyDP(thing, approx,0.15*Imgproc.arcLength(thing,true),true);
            if (!approx.empty()) {
                for (Point p : approx.toArray()) {
                    Imgproc.drawMarker(inputResized,p,new Scalar(2,20,200));
                }
            }
            if (approx.toArray().length<5) {
                Imgproc.drawMarker(inputResized,rect.center,new Scalar(200,20,2));
            }
        }

        Imgproc.drawMarker(gtresh,new Point(x,y),new Scalar(200,20,2));
        return inputResized;
    }
    public void setX(double x) {
        this.x=x;
    }
    public void setY(double y) {
        this.y=y;
    }
    public void setTelemetry(Telemetry tele) {
        this.telemetry=tele;
    }
    public boolean getColor() {
        return maxType;
    }
    public int getX() {
        if(maxContour!=null&& !maxContour.empty()) {
            try {
                MatOfPoint2f thing = new MatOfPoint2f(maxContour.toArray());
                RotatedRect rect = Imgproc.minAreaRect(thing);
                return rect.boundingRect().x;
            }
            catch (Exception e) {
                return 69;
            }
        }
        return -404;
    }
    public int getY() {
        if(maxContour!=null&& !maxContour.empty()) {
            try {
                MatOfPoint2f thing = new MatOfPoint2f(maxContour.toArray());
                RotatedRect rect = Imgproc.minAreaRect(thing);
                return rect.boundingRect().y;
            }
            catch (Exception e) {
                return 69;
            }
        }
        return -404;
    }

}
package org.firstinspires.ftc.teamcode;


import android.annotation.SuppressLint;
import android.util.Log;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PoleDetect extends OpenCvPipeline
{
    boolean viewportPaused;
    Mat dst = new Mat();
    Mat hsv = new Mat();
    Mat tresh = new Mat();
    Mat hierarchy = new Mat();
    Scalar color = new Scalar(0,255,0);
    int pos = 0;
    double maxArea = 0;
    double maxWidth = 0;
    double maxHeight = 0;
    int maxIndex = 0;
    MatOfPoint maxContour;
    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of c120ausing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */

    @Override
    public Mat processFrame(Mat input) {
        /*
         * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
         * will only dereference to the same image for the duration of this particular
         * invocation of this method. That is, if for some reason you'd like to save a copy
         * of this particular frame for later use, you will need to either clone it or copy
         * it to another Mat.
         */

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.GaussianBlur(input, dst, new Size(5,5), 0);
        Imgproc.cvtColor(dst, hsv, Imgproc.COLOR_RGB2HSV);
        // the values for the color scalar in this are hsv, h is 0-180,
        // so half the precision of a normal color wheel for hue,
        // then saturation and value go to 255,
        // you need to convert the normal 0-100 to 0-255
        Core.inRange(hsv, new Scalar(20,200,100), new Scalar(40,255,255), tresh);

        Imgproc.findContours(tresh, contours, hierarchy, Imgproc.RETR_TREE
                , Imgproc.CHAIN_APPROX_NONE);
        maxArea=0;
        maxContour=null;
        MatOfPoint2f contourMat = new MatOfPoint2f();
        RotatedRect maxRect = new RotatedRect();
        maxWidth = 0;
        maxHeight = 0;
        double width=0;
        double height=0;
        for(int i = 0; i < contours.size(); i++) {
            double area = Imgproc.contourArea(contours.get(i));
            try {
                contourMat = new MatOfPoint2f(contours.get(i).toArray());
                maxRect = Imgproc.minAreaRect(contourMat);
                width = maxRect.boundingRect().width;
                height = maxRect.boundingRect().height;
                Log.println(Log.INFO,"me","good job :D "+i+" "+contours.size());
            }
            catch(Exception e) {
                Log.println(Log.ERROR,"im still here >:D","aughhhh "+i+" "+contours.size());
            }
            if ( (area>800) && (area > maxArea && width < height / 2) ) {
                maxArea=area;
                maxWidth = width;
                maxContour=contours.get(i);
            }
            if(area>800){
                Imgproc.drawContours(input, contours, -1, color, 2
                        , Imgproc.LINE_8, hierarchy, 2, new Point());
                MatOfPoint point2 = contours.get(i);
                double[] position = point2.get(0,0);
                int x = (int) position[0];
                pos = x;
            }
        }
        if (maxContour!=null) {
            MatOfPoint2f thing = new MatOfPoint2f(maxContour.toArray());
            RotatedRect rect = Imgproc.minAreaRect(thing);
            MatOfPoint2f approx = new MatOfPoint2f();
            Imgproc.approxPolyDP(thing, approx,0.15*Imgproc.arcLength(thing,true),true);
            if (!approx.empty()) {
                for (Point p : approx.toArray()) {
                    Imgproc.drawMarker(input,p,new Scalar(2,20,200));
                }
            }
            if (approx.toArray().length<5) {
                Imgproc.drawMarker(input,rect.center,new Scalar(200,20,2));
            }
            else {
                maxContour=null;
            }

        }
        return input;
    }
    public int getPos() {
        return pos;
    }
    public MatOfPoint getMaxContour() {
        return maxContour;
    }
    public int getX() {
        if(maxContour!=null && !maxContour.empty()) {
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
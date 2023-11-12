package org.firstinspires.ftc.teamcode.poofyutils.processors;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;

public class PropProcessor implements VisionProcessor {

    private Mat hsv = new Mat();
    private Mat thresh = new Mat();
    private ArrayList<MatOfPoint> contours = new ArrayList<>();

    private Scalar purpleColor = new Scalar(50,50,3);

    public Scalar blueLower = new Scalar(93.5, 31.2, 121.8);
    public Scalar blueUpper = new Scalar(104.8, 143.1, 225.3);

    public Scalar redLower = new Scalar(0, 0, 0);
    public Scalar redUpper = new Scalar(0, 0, 0);

    public Rect leftROIBox = new Rect(150,140,25,25);
    public Rect centerROIBox = new Rect(320, 60, 25, 25);
    public Rect rightROIBox = new Rect(490, 100, 25, 25);

    public Mat leftMat = new Mat();
    public Mat centerMat = new Mat();
    public Mat rightMat = new Mat();

    public ArrayList<Mat> ROIs;

    public int spike = 0;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        ROIs = new ArrayList<>();
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {

        //filter to blue
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, blueLower, blueUpper, thresh);

        //create ROIs
        leftMat = new Mat(thresh, leftROIBox);
        centerMat = new Mat(thresh, centerROIBox);
        rightMat = new Mat(thresh, rightROIBox);

        //draw ROIs
        Imgproc.rectangle(input, leftROIBox, purpleColor);
        Imgproc.rectangle(input, centerROIBox, purpleColor);
        Imgproc.rectangle(input, rightROIBox, purpleColor);

        //left ROI
        Imgproc.findContours(leftMat, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(leftMat, contours, 0, purpleColor, 2);

        for (MatOfPoint contour : contours) {
            double width = calculateWidth(contour);
            double height = calculateHeight(contour);

            width -= 2;
            height -= 2;

            Moments moments = Imgproc.moments(contour);
            double cX = moments.get_m10() / moments.get_m00();
            double cY = moments.get_m01() / moments.get_m00();

            Imgproc.rectangle(
                    input,
                    new Point(cX - (width/2) + leftROIBox.x, cY - (height/2) + leftROIBox.y),
                    new Point(cX + (width/2) + leftROIBox.x, cY + (height/2) + leftROIBox.y),
                    new Scalar(240,240,240),
                    2
            );
        }
        Imgproc.putText(input, Integer.toString(contours.size()), new Point(leftROIBox.x,400), Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(255,255,255));
        if (contours.size() > 0) {
            spike = 1;
        }
        contours.clear();


        //center ROI
        Imgproc.findContours(centerMat, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(centerMat, contours, 0, purpleColor, 2);

        for (MatOfPoint contour : contours) {
            double width = calculateWidth(contour);
            double height = calculateHeight(contour);

            width -= 2;
            height -= 2;

            Moments moments = Imgproc.moments(contour);
            double cX = moments.get_m10() / moments.get_m00();
            double cY = moments.get_m01() / moments.get_m00();

            Imgproc.rectangle(
                    input,
                    new Point(cX - (width/2) + centerROIBox.x, cY - (height/2) + centerROIBox.y),
                    new Point(cX + (width/2) + centerROIBox.x, cY + (height/2) + centerROIBox.y),
                    new Scalar(240,240,240),
                    2
            );
        }
        Imgproc.putText(input, Integer.toString(contours.size()), new Point(centerROIBox.x,400), Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(255,255,255));
        if (contours.size() > 0) {
            spike = 2;
        }
        contours.clear();


        //right ROI
        Imgproc.findContours(rightMat, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(rightMat, contours, 0, purpleColor, 2);

        for (MatOfPoint contour : contours) {
            double width = calculateWidth(contour);
            double height = calculateHeight(contour);

            width -= 2;
            height -= 2;

            Moments moments = Imgproc.moments(contour);
            double cX = moments.get_m10() / moments.get_m00();
            double cY = moments.get_m01() / moments.get_m00();

            Imgproc.rectangle(
                    input,
                    new Point(cX - (width/2) + rightROIBox.x, cY - (height/2) + rightROIBox.y),
                    new Point(cX + (width/2) + rightROIBox.x, cY + (height/2) + rightROIBox.y),
                    new Scalar(240,240,240),
                    2
            );
        }
        Imgproc.putText(input, Integer.toString(contours.size()), new Point(rightROIBox.x,400), Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(255,255,255));
        if (contours.size() > 0) {
            spike = 3;
        }
        contours.clear();

        //clear the ROI mats
        ROIs.clear();

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    private double calculateWidth(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.width;
    }

    private double calculateHeight(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.height;
    }

    public int getSpike() {
        return spike;
    }
}

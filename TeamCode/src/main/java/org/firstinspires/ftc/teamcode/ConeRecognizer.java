package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

import org.apache.commons.math3.fraction.Fraction;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class ConeRecognizer extends OpenCvPipeline {
    ArrayList<MatOfPoint> coneContours = new ArrayList<>();
    ArrayList<MatOfPoint> infoContours = new ArrayList<>();
    Point coneCenter = null;
    Point infoCenter = null;
    Rect coneRect;
    Rect infoRect;


    // Camera Settings

    protected double centerX;
    protected double centerY;

    protected int imageWidth;
    protected int imageHeight;

    private double cameraPitchOffset;
    private double cameraYawOffset;

    private double fov;
    private double horizontalFocalLength;
    private double verticalFocalLength;
    private double verticalThreshold;

    boolean coneOnScreen = false;
    boolean infoOnScreen = false;

    public Point getConeCenter() {
        if(coneOnScreen) return coneCenter;
        else return null;
    }

    public ConeRecognizer(double fov, double cameraPitchOffset, double cameraYawOffset, double threshold) {
        super();
        this.fov = fov;
        this.cameraPitchOffset = cameraPitchOffset;
        this.cameraYawOffset = cameraYawOffset;
        this.verticalThreshold = threshold;
    }

    public ConeRecognizer(double fov) {
        this(fov, 0, 0, 15);
    }

    @Override
    public void init(Mat mat) {
        super.init(mat);

        imageWidth = mat.width();
        imageHeight = mat.height();

        // pinhole model calculations
        double diagonalView = Math.toRadians(this.fov);
        Fraction aspectFraction = new Fraction(this.imageWidth, this.imageHeight);
        int horizontalRatio = aspectFraction.getNumerator();
        int verticalRatio = aspectFraction.getDenominator();
        double diagonalAspect = Math.hypot(horizontalRatio, verticalRatio);
        double horizontalView = Math.atan(Math.tan(diagonalView / 2) * (horizontalRatio / diagonalAspect)) * 2;
        double verticalView = Math.atan(Math.tan(diagonalView / 2) * (verticalRatio / diagonalAspect)) * 2;
        horizontalFocalLength = this.imageWidth / (2 * Math.tan(horizontalView / 2));
        verticalFocalLength = this.imageHeight / (2 * Math.tan(verticalView / 2));
    }

    // A pipeline that finds and draws contours around the the ducks in the frame
    // and makes sure all of them are below a certain y value
    @Override
    public Mat processFrame(Mat input) {
        coneRect = null;
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lower = new Scalar(160, 100, 100);
        Scalar upper = new Scalar(180, 255, 255);
        Core.inRange(mat, lower, upper, mat);
        Imgproc.GaussianBlur(mat, mat, new org.opencv.core.Size(9, 9), 0);

        coneContours.clear();
        Imgproc.findContours(mat, coneContours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        for (int i = 0; i < coneContours.size(); i++) {
            Rect rect = Imgproc.boundingRect(coneContours.get(i));
            if (rect.width > 10 && rect.height > 10 && rect.y > verticalThreshold) {
                if (coneRect == null) {
                    coneRect = rect;
                } else if (rect.area() > coneRect.area()) {
                    coneRect = rect;
                }
            }
        }

        Imgproc.line(mat, new Point(0, verticalThreshold), new Point(mat.width(), verticalThreshold), new Scalar(255, 0, 0), 2);

        if (coneRect != null){
            Imgproc.rectangle(input, coneRect, new Scalar(0, 255, 0));
            coneCenter = getCenterofRect(coneRect);
            coneOnScreen = true;
        } else {
            coneOnScreen = false;
        }

        mat.release();
        processCone(mat, input, coneRect);
        return input;
    }
    public Mat processCone(Mat mat, Mat input, Rect coneRect) {
        Mat info = mat.submat(coneRect);
        infoRect = null;
        Imgproc.cvtColor(input, info, Imgproc.COLOR_RGB2HSV);

            Scalar lower = new Scalar(160, 100, 100);
            Scalar upper = new Scalar(180, 255, 255);
            Core.inRange(info, lower, upper, info);
            Imgproc.GaussianBlur(info, info, new org.opencv.core.Size(9, 9), 0);

            infoContours.clear();
            Imgproc.findContours(mat, infoContours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            for (int i = 0; i < coneContours.size(); i++) {
                Rect rect = Imgproc.boundingRect(infoContours.get(i));
                if (rect.width > 10 && rect.height > 10 && rect.y > verticalThreshold) {
                    if (infoRect == null) {
                        infoRect = rect;
                    } else if (rect.area() > infoRect.area()) {
                        infoRect = rect;
                    }
                }
            }
        Imgproc.line(mat, new Point(0, verticalThreshold), new Point(mat.width(), verticalThreshold), new Scalar(255, 0, 0), 2);

        if (infoRect != null){
            Imgproc.rectangle(input, infoRect, new Scalar(0, 255, 0));
            infoCenter = getCenterofRect(infoRect);
            infoOnScreen = true;
        } else {
            infoOnScreen = false;
        }
        Scalar purpleL = new Scalar(240, 100, 100);
        Scalar purpleH = new Scalar(260, 255, 255);
        Scalar greenL = new Scalar(50, 100, 100);
        Scalar greenH = new Scalar(70, 255, 255);
        Scalar orangeL = new Scalar(90, 100, 100);
        Scalar orangeH = new Scalar(110, 255, 255);
        if (testColor(mat, info, infoRect, purpleL, purpleH)) {
            boolean isPurple = true;
        }
        if (testColor(mat, info, infoRect, greenL, greenH)) {
            boolean isGreen = true;
        }
        if (testColor(mat, info, infoRect, orangeL, orangeH)) {
            boolean isOrange = true;
        }

        mat.release();
        return input;
    }
    public boolean isPurple(){
        return isPurple();
    }
    public boolean isGreen(){
        return isGreen();
    }
    public boolean isOrange(){
        return isOrange();
    }
    public boolean testColor(Mat mat, Mat info, Rect infoRect, Scalar lower, Scalar upper) {
        Core.inRange(info, lower, upper, info);
        Imgproc.GaussianBlur(info, info, new org.opencv.core.Size(9, 9), 0);

        infoContours.clear();
        Imgproc.findContours(mat, infoContours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        for (int i = 0; i < coneContours.size(); i++) {
            Rect rect = Imgproc.boundingRect(infoContours.get(i));
            if (rect.width > 10 && rect.height > 10 && rect.y > verticalThreshold) {
                if (infoRect == null) {
                    infoRect = rect;
                } else if (rect.area() > infoRect.area()) {
                    infoRect = rect;
                }
            }
        }
        Imgproc.line(mat, new Point(0, verticalThreshold), new Point(mat.width(), verticalThreshold), new Scalar(255, 0, 0), 2);

        return infoRect != null;
    }

    public Double calculateYaw(double offsetCenterX) {
        Double duckCenterX = null;
        if (coneCenter != null) {
            duckCenterX  = coneCenter.x;
        }
        if (coneOnScreen && duckCenterX != null) {
            return Math.atan((duckCenterX - offsetCenterX) / horizontalFocalLength);
        } else {
            return null;
        }
    }

    public Point getCenterofRect(@NonNull Rect rect) {
        return new Point(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);
    }
}
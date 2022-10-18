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
    boolean isPurple = false;
    boolean isGreen = false;
    boolean isOrange = false;


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
            processCone(input,coneRect);
        } else {
            coneOnScreen = false;
        }

        mat.release();
        return input;
    }
    public Mat processCone(Mat input, Rect coneRect) {
        Mat info = input.submat(coneRect);
        infoRect = null;
        Scalar lower = new Scalar(160, 100, 100);
        Scalar upper = new Scalar(180, 255, 255);
        Scalar purpleL = new Scalar(130, 100, 100);
        Scalar purpleH = new Scalar(150, 255, 255);
        Scalar greenL = new Scalar(40, 100, 100);
        Scalar greenH = new Scalar(70, 255, 255);
        Scalar orangeL = new Scalar(10, 100, 100);
        Scalar orangeH = new Scalar(30, 255, 255);
        try {
            if (testColor(info, purpleL, purpleH)) {
                isPurple = true;
            }
            if (testColor(info, greenL, greenH)) {
                isGreen = true;
            }
            if (testColor(info, orangeL, orangeH)) {
                isOrange = true;
            }
        } catch (Exception e) {
            System.out.println("error");
        }

        input.release();
        return info;
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
    public boolean testColor(Mat info, Scalar lower, Scalar upper) {
        Mat info_hsv = new Mat();
        Imgproc.cvtColor(info, info_hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(info_hsv, lower, upper, info_hsv);
        double info_val = 0;
        for (int i = 0; i < 5; i++) {
            info_val = info_val+ Core.sumElems(info_hsv).val[i];
        }
        return info_val > 5;

//        infoContours.clear();
//        Imgproc.findContours(info_hsv, infoContours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
//        for (int i = 0; i < coneContours.size(); i++) {
//            Rect rect = Imgproc.boundingRect(infoContours.get(i));
//            if (rect.width > 10 && rect.height > 10 && rect.y > verticalThreshold) {
//                if (infoRect == null) {
//                    infoRect = rect;
//                } else if (rect.area() > infoRect.area()) {
//                    infoRect = rect;
//                }
//            }
//        }
//        Imgproc.line(info_hsv, new Point(0, verticalThreshold), new Point(info_hsv.width(), verticalThreshold), new Scalar(255, 0, 0), 2);
//
//        return infoRect != null;
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
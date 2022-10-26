package org.firstinspires.ftc.teamcode.opencv;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;

public class ColorPipeline extends OpenCvPipeline {
    ArrayList<MatOfPoint> orangeContours = new ArrayList<>();
    int orangeIndex;
    ArrayList<MatOfPoint> greenContours = new ArrayList<>();
    int greenIndex;
    SignalColor color = SignalColor.UNSET;

    @Override
    public Mat processFrame(Mat input) {
        Mat orangeMat = new Mat();
        Mat greenMat = new Mat();

        Imgproc.cvtColor(input,orangeMat,Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input,greenMat,Imgproc.COLOR_RGB2HSV);

        Imgproc.GaussianBlur(orangeMat,orangeMat,new Size(5,5),0);
        Imgproc.GaussianBlur(greenMat,greenMat,new Size(5,5),0);

        Scalar orangeLower = new Scalar(0,100,100);
        Scalar orangeUpper = new Scalar(7,255,200);

        Scalar greenLower = new Scalar(70,0,0);
        Scalar greenUpper = new Scalar(80,255,200);

        Core.inRange(orangeMat,orangeLower,orangeUpper,orangeMat);
        Core.inRange(greenMat,greenLower,greenUpper,greenMat);

        Imgproc.morphologyEx(orangeMat,orangeMat,Imgproc.MORPH_OPEN,Mat.ones(new Size(3,3), CvType.CV_32F));
        Imgproc.morphologyEx(greenMat,greenMat,Imgproc.MORPH_OPEN,Mat.ones(new Size(3,3), CvType.CV_32F));

        orangeContours.clear();
        greenContours.clear();
        MatOfPoint workingContour = null;
        Imgproc.findContours(orangeMat, orangeContours, new Mat(), Imgproc.RETR_LIST,Imgproc.CHAIN_APPROX_TC89_KCOS);
        Imgproc.findContours(greenMat, greenContours, new Mat(), Imgproc.RETR_LIST,Imgproc.CHAIN_APPROX_TC89_KCOS);

        for (int i = 0; i < orangeContours.size(); i++) {
            MatOfPoint newContour = orangeContours.get(i);
            if(Imgproc.boundingRect(newContour).width >10 && Imgproc.boundingRect(newContour).height > 10){
                if(workingContour == null){
                    workingContour = newContour;
                    orangeIndex =i;
                }else if(Imgproc.contourArea(newContour)> Imgproc.contourArea(workingContour)){
                    workingContour = newContour;
                    orangeIndex = i;
                }
            }
        }
        for (int i = 0; i < greenContours.size(); i++) {
            MatOfPoint newContour = greenContours.get(i);
            if(Imgproc.boundingRect(newContour).width >10 && Imgproc.boundingRect(newContour).height > 10){
                if(workingContour == null){
                    workingContour = newContour;
                    greenIndex =i;
                }else if(Imgproc.contourArea(newContour)> Imgproc.contourArea(workingContour)){
                    workingContour = newContour;
                    greenIndex = i;
                }
            }
        }


        try {
            Imgproc.drawContours(input,orangeContours,orangeIndex,new Scalar(170, 100, 80));
        }catch(Exception ignored){
            System.out.printf("Orange Contour not found");
        }

        try {
            Imgproc.drawContours(input,greenContours,greenIndex,new Scalar(0,255,0));
        }catch(Exception ignored){
            System.out.println("No green contours");
        }

        //pick the color with the largest area if neither exceeds CVConstants.MINIMUM_CONTOUR_AREA then set color to Purple
        double orangeArea;
        try{
            orangeArea = Imgproc.contourArea(orangeContours.get(orangeIndex));
            if(orangeArea < CVConstants.MINIMUM_CONTOUR_AREA){
                orangeArea = 0;
            }
        } catch (Exception e){
            System.out.println("No orange contours");
            orangeArea = 0;
        }

        double greenArea;
        try{
            greenArea = Imgproc.contourArea(greenContours.get(greenIndex));
            if(greenArea < CVConstants.MINIMUM_CONTOUR_AREA){
                greenArea = 0;
            }
        } catch (Exception e){
            System.out.println("No green contours");
            greenArea = 0;
        }
        double purpleArea = 1;

        if(orangeArea > purpleArea && orangeArea > greenArea) {
            color = SignalColor.ORANGE;
        } else if(greenArea > purpleArea) {
            color = SignalColor.GREEN;
        } else {
            color = SignalColor.PURPLE;
        }

        greenMat.release();
        orangeMat.release();
        return input;
    }

    public SignalColor getColor() {
        return color;
    }
}
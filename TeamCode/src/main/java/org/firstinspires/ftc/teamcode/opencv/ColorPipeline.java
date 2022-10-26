package org.firstinspires.ftc.teamcode.opencv;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;

public class ColorPipeline extends OpenCvPipeline {
    ArrayList<MatOfPoint> orangeContours = new ArrayList<>();
    Integer orangeIndex;
    ArrayList<MatOfPoint> greenContours = new ArrayList<>();
    Integer greenIndex;
    SignalColor color = SignalColor.UNSET;
    ArrayList<MatOfPoint> purpleContours = new ArrayList<>();
    Integer purpleIndex;
    Telemetry telemetry;

    public ColorPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {

        Scalar orangeLower = new Scalar(0,100,100);
        Scalar orangeUpper = new Scalar(7,255,200);

        Scalar greenLower = new Scalar(50,0,0);
        Scalar greenUpper = new Scalar(60,255,255);

        Scalar purpleLower = new Scalar(160,0,0);
        Scalar purpleUpper = new Scalar(180,255,255);

        double orangeArea = findColorContourArea(input, orangeContours, orangeLower, orangeUpper, new Scalar(255, 100, 0));
        double greenArea = findColorContourArea(input, greenContours, greenLower, greenUpper, new Scalar(0, 255, 0));
        double purpleArea = findColorContourArea(input, purpleContours, purpleLower, purpleUpper, new Scalar(255, 0, 255));


        //pick the color with the largest area
        if(orangeArea > purpleArea && orangeArea > greenArea) {
            color = SignalColor.ORANGE;
        } else if(greenArea > purpleArea) {
            color = SignalColor.GREEN;
        } else {
            color = SignalColor.PURPLE;
        }

        telemetry.addData("Color", color);
        telemetry.addData("Orange Area", orangeArea);
        telemetry.addData("Green Area", greenArea);
        telemetry.addData("Purple Area", purpleArea);
        telemetry.update();
        return input;
    }

    private double findColorContourArea(Mat input, ArrayList<MatOfPoint> contours, Scalar lower, Scalar upper, Scalar color){
        Mat workingMat = new Mat();
        //convert to HSV
        Imgproc.cvtColor(input,workingMat,Imgproc.COLOR_RGB2HSV);
        //blur the mat
        Imgproc.GaussianBlur(workingMat,workingMat,new Size(5,5),0);
        //filter the mat
        Core.inRange(workingMat,lower,upper,workingMat);
        //morph the mat (remove noise)
        Imgproc.morphologyEx(workingMat,workingMat,Imgproc.MORPH_OPEN,Mat.ones(new Size(3,3), CvType.CV_32F));
        //find the contours
        contours.clear();
        Imgproc.findContours(workingMat, contours, new Mat(), Imgproc.RETR_LIST,Imgproc.CHAIN_APPROX_TC89_KCOS);
        MatOfPoint workingContour = null;
        //find the largest contour
        int index = 0;
        for (int i = 0; i < contours.size(); i++) {
            MatOfPoint newContour = contours.get(i);
            if(Imgproc.boundingRect(newContour).width >10 && Imgproc.boundingRect(newContour).height > 10){
                if(workingContour == null){
                    workingContour = newContour;
                    index =i;
                }else if(Imgproc.contourArea(newContour)> Imgproc.contourArea(workingContour)){
                    workingContour = newContour;
                    index = i;
                }
            }
        }
        //release the mat
        workingMat.release();
        //add the contour to the image
        try {
            Imgproc.drawContours(input,contours,index,color);
            return Imgproc.contourArea(workingContour);
        }catch(Exception ignored){
            System.out.printf("Orange Contour not found");
            return 0;
        }

    }

    public SignalColor getColor() {
        return color;
    }
}
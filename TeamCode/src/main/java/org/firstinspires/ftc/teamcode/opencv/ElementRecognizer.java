package org.firstinspires.ftc.teamcode.opencv;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class ElementRecognizer extends OpenCvPipeline {
    private int parkingArea = 3;
    double purpleValue;
    double greenValue;
    public int getParkingArea() {
        return parkingArea;
    }

    public double getPurpleValue() {
        return purpleValue;
    }

    public double getGreenValue() {
        return greenValue;
    }

    // Recognizes the shipping hub level based on where the team shipping element is located
    // Create two possible boxes it can be in
    static final Rect BOX = new Rect(
            new Point(0, 0),
            new Point(200, 200)
    );

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();
        Scalar purpleL = new Scalar(110, 100, 35); // purple lower in hsv
        Scalar purpleH = new Scalar(150, 255, 255); // purple upper in hsv
        Scalar greenL = new Scalar(110, 100, 35); // green lower in hsv
        Scalar greenH = new Scalar(150, 255, 255); // green upper in hsv
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV); // convert to hsv

        Mat left = mat.submat(BOX);
        Mat right = mat.submat(BOX);

        Core.inRange(input, purpleL, purpleH, left); // make purple white, everything else black
        Core.inRange(input, greenL, greenH, right); // make green white, everything else black

        purpleValue = Core.sumElems(left).val[0] / BOX.area(); // Get white pixel / total pixel count
        greenValue = Core.sumElems(right).val[0] / BOX.area(); // Get white pixel / total pixel count

        Imgproc.rectangle(mat, BOX, new Scalar(255, 0, 0), 2); // draw rectangles around the boxes
        Imgproc.rectangle(mat, BOX, new Scalar(255, 0, 0), 2);
        // If neither value is high enough , then the shipping hub level is 3, otherwise we continue
        if (purpleValue > 1 || greenValue > 1){
            if (purpleValue >= greenValue){
                parkingArea = 1;
                Imgproc.rectangle(mat, BOX, new Scalar(0, 255, 0), 2);
            } else if (purpleValue < greenValue) {
                parkingArea = 2;
                Imgproc.rectangle(mat, BOX, new Scalar(0, 255, 0), 2);
            }
        } else {
            parkingArea = 3;
        }

        // I think this stops a memory leak?
        left.release();
        right.release();
        return mat;
    }
}


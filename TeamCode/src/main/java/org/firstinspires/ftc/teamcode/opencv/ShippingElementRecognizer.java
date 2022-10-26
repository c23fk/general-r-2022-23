package org.firstinspires.ftc.teamcode.opencv;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ShippingElementRecognizer extends OpenCvPipeline {
    private int shippingHubLevel = 3;
    double imgValue;
    public int getShippingHubLevel() {
        return shippingHubLevel;
    }
    public double getValue() {
        return imgValue;
    }

    // Recognizes the shipping hub level based on where the team shipping element is located
    // Create two possible boxes it can be in
    static final Rect BOX = new Rect(
            new Point(120, 60),
            new Point(360, 580)
    );

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();
        Scalar lowHSV = new Scalar(160, 100, 100); // purple lower in hsv
        Scalar highHSV = new Scalar(179, 255, 255); // purple upper in hsv
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV); // convert to hsv
        Core.inRange(input, lowHSV, highHSV, mat); // make purple white, everything else black
        Mat img = mat.submat(BOX);

        imgValue = Core.sumElems(img).val[0] / BOX.area(); // Get white pixel / total pixel count
        Imgproc.rectangle(mat, BOX, new Scalar(255, 0, 0), 2);
        Scalar scalar = new Scalar(0, 0, 0);
        // If neither value is high enough , then the shipping hub level is 3, otherwise we continue
//        if (leftValue > 10 || rightValue > 10){
//            if (leftValue >= rightValue){
//                shippingHubLevel = 1;
//                Imgproc.rectangle(mat, LEFTBOX, new Scalar(0, 255, 0), 2);
//            } else if (leftValue < rightValue) {
//                shippingHubLevel = 2;
//                Imgproc.rectangle(mat, RIGHTBOX, new Scalar(0, 255, 0), 2);
//            }
//        } else {
//            shippingHubLevel = 3;
//        }
        if ((imgValue) > 0){
            shippingHubLevel = 1;
            }
        else {
            shippingHubLevel = 0;
        }

        // I think this stops a memory leak?
        img.release();
        return mat;
    }
}

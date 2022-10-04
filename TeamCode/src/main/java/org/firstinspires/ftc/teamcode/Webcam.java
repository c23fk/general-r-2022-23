package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class Webcam {
    OpenCvWebcam webcam;
    OpenCvWebcam frontWebcam;
    OpenCvPipeline activePipeline;
    ShippingElementRecognizer shippingElementRecognizer;
    ConeRecognizer coneRecognizer;

    public int getShippingHubLevel() {
        return shippingElementRecognizer.getShippingHubLevel();
    }

    public void switchToDuckPipeline(){
        frontWebcam.setPipeline(coneRecognizer);
        activePipeline = coneRecognizer;
    }

    public Point getDuckCenter() {
        return coneRecognizer.getDuckCenter();
    }

    public double getDuckAngle() {
        return -coneRecognizer.calculateYaw(Constants.CAMERA_POSITION);
    }

    public Double calculateYaw(double cameraPosition) {
        if (activePipeline == coneRecognizer) {
            return coneRecognizer.calculateYaw(cameraPosition);
        } else {
            return null;
        }
    }

    public void init(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        2, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY); //Whether to split the container vertically or horizontally
        // Setup first camera
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), viewportContainerIds[0]);
        shippingElementRecognizer = new ShippingElementRecognizer();
        webcam.setPipeline(shippingElementRecognizer);
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
                // This will be called if the camera could not be opened
            }
        });

        // Second camera
        frontWebcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Front Webcam"), viewportContainerIds[1]);
        coneRecognizer = new ConeRecognizer(78);
        frontWebcam.setPipeline(coneRecognizer);
        activePipeline = coneRecognizer;
        frontWebcam.setMillisecondsPermissionTimeout(2500);
        frontWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                frontWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // This will be called if the camera could not be opened
            }
        });
    }
}
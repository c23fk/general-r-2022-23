package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.opencv.ColorPipeline;
import org.firstinspires.ftc.teamcode.opencv.ColorTags;
import org.firstinspires.ftc.teamcode.opencv.SignalColor;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class Camera_Array implements Mechanism{
    private final Telemetry telemetry;
    private OpenCvWebcam cam1;
    private ColorTags pipeline1;
    private OpenCvWebcam cam2;
    private ColorTags pipeline2;
    private int tag;
    private boolean initialized = false;
    private boolean focusCam1 = true;

    private LinkedList<Integer> tags = new LinkedList<>();

    public Camera_Array(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    @Override
    public void init(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId,2, OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY);
        cam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);
        pipeline1 = new ColorTags(Constants.tagsize,Constants.fx,Constants.fy,Constants.cx,Constants.cy);
        cam1.setPipeline(pipeline1);
        cam1.setMillisecondsPermissionTimeout(2500);
        cam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cam1.startStreaming(Constants.CAM_WIDTH, Constants.CAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                //telemetry.addData("Camera status:", "initialized");
            }

            @Override
            public void onError(int errorCode) {
                // This will be called if the camera could not be opened
            }
        });
        cam2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[1]);
        pipeline2 = new ColorTags(Constants.tagsize,Constants.fx,Constants.fy,Constants.cx,Constants.cy);
        cam2.setPipeline(pipeline2);
        cam2.setMillisecondsPermissionTimeout(2500);
        cam2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cam2.startStreaming(Constants.CAM_WIDTH, Constants.CAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                //telemetry.addData("Camera status:", "initialized");
            }

            @Override
            public void onError(int errorCode) {
                // This will be called if the camera could not be opened
            }
        });
        while(!pipeline1.initialized()|| !pipeline2.initialized()){
            telemetry.addData("cameras:", "initializing");
            telemetry.update();
        }
        telemetry.addData("cameras:", "ready");
        telemetry.update();
        initialized = true;
    }

    @Override
    public void run(Gamepad gamepad) {
        ArrayList<AprilTagDetection> pip1 = pipeline1.getLatestDetections();
        ArrayList<AprilTagDetection> pip2 = pipeline2.getLatestDetections();
        if(pip1.size() != 0) {
            tags.add(pip1.get(0).id);
        }
        if(pip2.size() != 0) {
            tags.add(pip2.get(0).id);
        }
        while(tags.size()>100){
            tags.removeFirst();
        }
        try {
            tag = mostCommon(tags);
        }catch(Exception e){
            tag = 4;
        }
        telemetry.addData("Cam_1 Tag:", pipeline1.getLatestDetections().size() == 0? "none":pipeline1.getLatestDetections().get(0).id);
        telemetry.addData("Cam_2 Tag:", pipeline2.getLatestDetections().size() == 0? "none":pipeline2.getLatestDetections().get(0).id);
        telemetry.addData("Most Common:", mostCommon(tags));
        telemetry.addData("length", tags.size());

    }
    public int getTag(){
        return tag;
    }

    // This is copied from stack overflow
    public static <T> T mostCommon(List<T> list) {
        if(list.isEmpty()) {
            return null;
        }
        Map<T, Integer> map = new HashMap<>();
        for (T t : list) {
            Integer val = map.get(t);
            map.put(t, val == null ? 1 : val + 1);
        }
        Map.Entry<T, Integer> max = null;
        for (Map.Entry<T, Integer> e : map.entrySet()) {
            if (max == null || e.getValue() > max.getValue())
                max = e;
        }
        assert max != null;
        return max.getKey();
    }

    public double getYellowLocation(){
        if(pipeline1.getYellowArea()>pipeline2.getYellowArea()) {
            focusCam1 = true;
            return pipeline1.getYellowLocation();
        }
        focusCam1 = false;
        return pipeline2.getYellowLocation();
    }
    public boolean initialized(){
        return initialized;
    }

    public boolean cam1Focus(){
        return focusCam1;
    }

    public double yellowPos(int cam){
        if(cam == 1){
            return pipeline1.getYellowLocation();
        }
        return pipeline2.getYellowLocation();
    }

    public double yellowArea(int cam){
        if(cam == 1){
            return pipeline1.getYellowArea();
        }
        return pipeline2.getYellowArea();
    }

    public OpenCvWebcam getCamera(int cam){
        if(cam == 1){
            return cam1;
        }
        return cam2;
    }

    public double calculateMovement(){
        double yellowX = getYellowLocation();
        double thatThing = 0.01;//smallifys the numbers
        ColorTags cam = focusCam1?pipeline1:pipeline2;
        if(cam.getYellowArea() == 0) {
            return 0;
        }
        double alignment = focusCam1?235*2:100*2;
        return (alignment-yellowX)/(Constants.CAM_WIDTH*2) * thatThing;
    }

}

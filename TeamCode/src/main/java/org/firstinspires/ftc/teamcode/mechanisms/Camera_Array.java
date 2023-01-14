package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.opencv.ColorPipeline;
import org.firstinspires.ftc.teamcode.opencv.SignalColor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class Camera_Array implements Mechanism{
    private final Telemetry telemetry;
    private OpenCvWebcam cam1;
    private ColorPipeline pipeline1;
    private OpenCvWebcam cam2;
    private ColorPipeline pipeline2;
    private SignalColor color;
    private boolean initialized = false;
    private boolean focusCam1 = true;

    private LinkedList<SignalColor> colors = new LinkedList<>();

    public Camera_Array(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    @Override
    public void init(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId,2, OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY);
        cam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);
        pipeline1 = new ColorPipeline(telemetry);
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
        pipeline2 = new ColorPipeline(telemetry);
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
        while(pipeline1.getColor() == SignalColor.INACTIVE|| pipeline2.getColor() == SignalColor.INACTIVE){
            telemetry.addData("cameras:", "initializing");
            telemetry.update();
        }
        telemetry.addData("cameras:", "ready");
        telemetry.update();
        initialized = true;
    }

    @Override
    public void run(Gamepad gamepad) {
        if(pipeline1.getColor() != SignalColor.UNSET||pipeline2.getColor() != SignalColor.UNSET) {
            if(pipeline1.getMaxArea()>pipeline2.getMaxArea()){
                colors.add(pipeline1.getColor());
            }else{
                colors.add(pipeline2.getColor());
            }
            if(colors.size() > 100) {
                colors.removeFirst();
            }
        }
        if (colors.size() > 100) {
            colors.removeFirst();
        }
        color = mostCommon(colors);
        telemetry.addData("Cam_1 Color:", pipeline1.getColor());
        telemetry.addData("Cam_2 Color:", pipeline2.getColor());
        telemetry.addData("Most Common:", mostCommon(colors));
        telemetry.addData("length", colors.size());

    }
    public SignalColor getColor(){
        return color;
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
            return pipeline1.getYellowLocation();
        }
        return pipeline2.getYellowLocation();
    }

    public double calculateMovement(){
        double yellowX = getYellowLocation();
        double thatThing = 0.0025;//smallifys the numbers
        ColorPipeline cam = focusCam1?pipeline1:pipeline2;
        if(cam.getYellowArea() == 0) {
            return 0;
        }
        double alignment = focusCam1?440:170;
        return (alignment-yellowX)/Constants.CAM_WIDTH * thatThing;
    }

}

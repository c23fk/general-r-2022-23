package org.firstinspires.ftc.teamcode.mechanisms;

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

public class Camera_RB2 implements Mechanism{
    private OpenCvWebcam webcam;
    private ColorPipeline pipeline;
    private SignalColor color;
    private final Telemetry telemetry;
    private double largestArea = 0;
    private boolean initialized = false;
    private double yellowArea = 0;
    private LinkedList<SignalColor> colors = new LinkedList<>();

    public Camera_RB2(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    @Override
    public void init(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new ColorPipeline(telemetry);
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(Constants.CAM_WIDTH, Constants.CAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                //telemetry.addData("Camera status:", "initialized");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera status:", "error");
                // This will be called if the camera could not be opened
            }
        });
        double startTime = System.currentTimeMillis();
        while(pipeline.getColor() == SignalColor.UNSET && System.currentTimeMillis() - startTime < 3000){
            telemetry.addData("camera ready?", initialized);
            telemetry.update(); // Update telemetry
        }
        initialized = true;
        telemetry.addData("camera ready?", initialized);
        telemetry.update();
    }

    public void init(HardwareMap hardwareMap, String number) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam "+number), cameraMonitorViewId);
        pipeline = new ColorPipeline(telemetry);
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(Constants.CAM_WIDTH, Constants.CAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                //telemetry.addData("Camera status:", "initialized");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera status:", "error");
                // This will be called if the camera could not be opened
            }
        });
        double startTime = System.currentTimeMillis();
        while(pipeline.getColor() == SignalColor.UNSET && System.currentTimeMillis() - startTime < 3000){
            telemetry.addData("camera ready?", initialized);
            telemetry.update(); // Update telemetry
        }
        initialized = true;
        telemetry.addData("camera ready?", initialized);
        telemetry.update();
    }

    @Override
    public void run(Gamepad gamepad) {

        if(pipeline.getColor() != SignalColor.UNSET) {
            colors.add(pipeline.getColor());
            largestArea = pipeline.getMaxArea();
            if(colors.size() > 100) {
                colors.removeFirst();
            }
        }
        if (colors.size() > 100) {
            colors.removeFirst();
        }
        telemetry.addData("Color:", pipeline.getColor());
        telemetry.addData("MostCommon:", mostCommon(colors));
        telemetry.addData("length", colors.size());
        telemetry.update();
        color = mostCommon(colors);
    }
    public SignalColor getColor(){
        return color;
    }

    public double getYellowArea(){
        return pipeline.getYellowArea();
    }

    public double getLargestArea(){ return largestArea; }

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

    public boolean initialized(){
        return initialized;
    }

    public double getYellowLocation(){
        return pipeline.getYellowLocation();
    }
}

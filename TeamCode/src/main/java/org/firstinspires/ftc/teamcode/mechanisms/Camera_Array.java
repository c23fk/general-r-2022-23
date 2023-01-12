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

public class Camera_Array implements Mechanism{
    private final Telemetry telemetry;
    private Camera_RB2 cam1 = null;
    private Camera_RB2 cam2 = null;
    private SignalColor color;
    private boolean initialized = false;
    private boolean focusCam1 = true;
    private LinkedList<SignalColor> colors = new LinkedList<>();

    public Camera_Array(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    @Override
    public void init(HardwareMap hardwareMap) {
        cam1 = new Camera_RB2(telemetry);
        cam2 = new Camera_RB2(telemetry);
        cam1.init(hardwareMap, "1");
        cam2.init(hardwareMap, "2");
        while(!cam1.initialized() || ! cam2.initialized()){}
        initialized = true;
    }

    @Override
    public void run(Gamepad gamepad) {
        if(cam1.getColor() != SignalColor.UNSET||cam2.getColor() != SignalColor.UNSET) {
            if(cam1.getLargestArea()>cam2.getLargestArea()){
                colors.add(cam1.getColor());
            }else{
                colors.add(cam2.getColor());
            }
            if(colors.size() > 100) {
                colors.removeFirst();
            }
        }
        if (colors.size() > 100) {
            colors.removeFirst();
        }
        telemetry.addData("Cam_1 Color:", cam1.getColor());
        telemetry.addData("Cam_2 Color:", cam2.getColor());
        telemetry.addData("Most Common:", mostCommon(colors));
        telemetry.addData("length", colors.size());
        telemetry.update();
        color = mostCommon(colors);
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
        if(cam1.getYellowArea()>cam2.getYellowArea()) {
            focusCam1 = true;
            return cam1.getYellowLocation();
        }
        focusCam1 = false;
        return cam2.getYellowLocation();
    }
    public boolean initialized(){
        return initialized;
    }

    public boolean cam1Focus(){
        return focusCam1;
    }

    public double calculateMovement(){
        double thatThing = 0.01;//smallifys the numbers
        Camera_RB2 cam = focusCam1?cam1:cam2;
        double alignment = focusCam1?440:120;
        return (alignment-cam.getYellowLocation())/Constants.CAM_WIDTH * thatThing;
    }

}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opencv.ColorPipeline;
import org.firstinspires.ftc.teamcode.opencv.ElementRecognizer;
import org.firstinspires.ftc.teamcode.opencv.SignalColor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;


@Autonomous(name = "TestAuto2")
public class TestAuto2 extends LinearOpMode {

    /**
     * Amount of time elapsed
     */
    private ElapsedTime runtime = new ElapsedTime();
    RobotHardware robot = new RobotHardware();
    Thread telemetryHandler = new Thread(){
        @Override
        public void run() {
            while (opModeIsActive()) {
                telemetry.addData("Runtime(s): ", runtime.seconds());
                telemetry.addData("Color: ", robot.getColor());
                telemetry.update();
            }
        }
    };
    private int parkingLocation;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();
        robot.init(hardwareMap, telemetry);
        LinkedList<SignalColor> levels = new LinkedList<>();
        // Make the level the most common one from the past 100 loops
        while (!isStarted() && !isStopRequested()) {
            if(robot.getColor() != SignalColor.UNSET) {
                levels.add(robot.getColor());
                if(levels.size() > 100) {
                    levels.removeFirst();
                }
            }
            if (levels.size() > 100) {
                levels.removeFirst();
            }
            telemetry.addData("Color", robot.getColor());
            telemetry.addData("length", levels.size());
            telemetry.update();
        }
        SignalColor level = mostCommon(levels);
        if(level == null) {
            level = SignalColor.UNSET;
        }
        telemetry.addData("Level", level);
        switch (level) {
            case PURPLE:
                parkingLocation = 1;
                break;
            case GREEN:
                parkingLocation = 3;
                break;
            default:
                parkingLocation = 2;
                break;
        }
        waitForStart();
        telemetryHandler.start();

        robot.forwardDrive(0.5);
        while(robot.getBackDistance()<50&&opModeIsActive()){}
        robot.forwardDrive(0.25);
        while(robot.getBackDistance()<75&&opModeIsActive()){}
        robot.stopDrive();
        if(parkingLocation == 1) {
            robot.strafeRight(0.5,-900,0.5);
        }
        if(parkingLocation == 3) {
            robot.strafeRight(0.5,900,0.5);
        }
        sleep(1500);
        robot.setSlidePosition(Constants.INTAKE_POSITION);
        runtime.reset();
    }


    public static void resetEncoder(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        return max.getKey();
    }
}


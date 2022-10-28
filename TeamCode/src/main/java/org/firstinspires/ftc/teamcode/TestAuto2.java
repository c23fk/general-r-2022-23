package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opencv.SignalColor;

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
                telemetry.addData("Left Distance", robot.getLeftDistance());
                telemetry.addData("Right Distance", robot.getRightDistance());
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
        robot.wristUp();
        robot.closeClaw();


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


        robot.wristUp();
        robot.closeClaw();


        robot.forwardDrive(0.1);
        while(robot.getBackDistance()<75&&opModeIsActive()){}
        robot.stopDrive();
        robot.closeClaw();

        robot.rotateLeft(0.2, 180, 10);

//        robot.strafeRight(-0.1);
//        while(robot.getRightDistance()<73&&opModeIsActive()){}
//        robot.stopDrive();
//
//        robot.strafeRight(0.1);
//        while(robot.getRightDistance()>73&&opModeIsActive()){}
//        robot.stopDrive();

        robot.setSlidePosition(3400);
        sleep(2000);
        telemetry.addData("Left Distance", robot.getLeftDistance());
//        robot.strafeLeft(0.1);
//        while(robot.getLeftDistance() < 70 &&opModeIsActive()){}
        robot.stopDrive();

        robot.forwardDrive(0.1);
        while(robot.getBackDistance()<66&&opModeIsActive()){}
        robot.stopDrive();


        sleep(2500);
        robot.wristDown();
        sleep(1000);
        robot.openClaw();
        sleep(1500);
        robot.closeClaw();

        if(parkingLocation == 1) {
            robot.strafeRight(0.1,-900,0.5);
        }
        if(parkingLocation == 3) {
            robot.strafeRight(0.1,900,0.5);
        }
        sleep(1500);
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


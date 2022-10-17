package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.LinkedList;


@Autonomous(name = "TestAuto")
public class TestAuto extends LinearOpMode {

    /**
     * Amount of time elapsed
     */
    private ElapsedTime runtime = new ElapsedTime();
    OpenCvWebcam webcam;

    public static void resetEncoder(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing");

        telemetry.addData("Status", "Initialized");

        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam1"), cameraMonitorViewId);
        ConeRecognizer pipeline = new ConeRecognizer(5);
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {


            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
                telemetry.addData("Status", "Webcam on");
                telemetry.update();

            }

            @Override
            public void onError(int errorCode) {
                // This will be called if the camera could not be opened
            }
        });

        waitForStart();
        //lift encoder numbers
        //bottom is 1200
        // middle is 3200

        runtime.reset();

//        int level;
//        int[] counts = {0,0,0};
//        for(int i=0;i<50;i++) {
//            if(pipeline.getShippingHubLevel() == 0) {
//                i = 0;
//                continue;
//            }
//            counts[pipeline.getShippingHubLevel() - 1] ++;
//        }
//
//        if(counts[0] > counts[1] && counts[0] > counts[2]) {
//            level = 1;
//        } else if(counts[1] > counts[0] && counts[1] > counts[2]) {
//            level = 2;
//        } else {
//            level = 3;
//        }
        String color = null;
        if (pipeline.isPurple()) {
            color = "Purple";
        }
        if (pipeline.isGreen()) {
            color = "Green";
        }
        if (pipeline.isOrange()) {
            color = "Orange";
        }
        telemetry.addData("Color:", color);
        telemetry.update();
//        int level = pipeline.getShippingHubLevel();
//        telemetry.addData("Shipping Hub Level", level);
//        telemetry.update();
//

//
//        telemetry.addData("Shipping Hub Level", level);
//        telemetry.update();

        // Drive to the the shipping hub


        // Deposit the box on the correct level

//       encoder auto
//
//        if(level == 1) {
//
//            telemetry.addData("Shipping Hub Level", level);
//            telemetry.update();
//            Thread.sleep(700);
//        } else if (level == 2) {
//
//            telemetry.addData("Shipping Hub Level", level);
//            telemetry.update();
//            Thread.sleep(1000);
//        } else {
//
//            telemetry.addData("Shipping Hub Level", level);
//            telemetry.update();
//            Thread.sleep(1000);
//        }







//
//       turnClockWiseByTime(1000,-1);
//       driveForwardByTime(2000, 1);
    }
}


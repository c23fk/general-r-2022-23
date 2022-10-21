package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.CAM_HEIGHT;
import static org.firstinspires.ftc.teamcode.Constants.CAM_WIDTH;

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
        ElementRecognizer pipeline = new ElementRecognizer();
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {


            @Override
            public void onOpened() {
                webcam.startStreaming(Constants.CAM_WIDTH, Constants.CAM_HEIGHT, OpenCvCameraRotation.SIDEWAYS_LEFT);
                telemetry.addData("Status", "Webcam on");
                telemetry.update();

            }

            @Override
            public void onError(int errorCode) {
                // This will be called if the camera could not be opened
            }
        });
        while(pipeline == null){}
        waitForStart();
        //lift encoder numbers
        //bottom is 1200
        // middle is 3200

        runtime.reset();

        int area;
        int[] counts = {0,0,0};
        for(int i=0;i<50;i++) {
            if(pipeline.getParkingArea() == 0) {
                i = 0;
                continue;
            }
            counts[pipeline.getParkingArea() - 1] ++;
        }

        if(counts[0] > counts[1] && counts[0] > counts[2]) {
            area = 1;
        } else if(counts[1] > counts[0] && counts[1] > counts[2]) {
            area = 2;
        } else {
            area = 3;
        }
        telemetry.addData("Parking Area: ", area);
        telemetry.update();
//        int area = pipeline.getShippingHubLevel();
//        telemetry.addData("Shipping Hub Level", area);
//        telemetry.update();
//

//
//        telemetry.addData("Shipping Hub Level", area);
//        telemetry.update();

        // Drive to the the shipping hub


        // Deposit the box on the correct area

//       encoder auto
//
//        if(area == 1) {
//
//            telemetry.addData("Shipping Hub Level", area);
//            telemetry.update();
//            Thread.sleep(700);
//        } else if (area == 2) {
//
//            telemetry.addData("Shipping Hub Level", area);
//            telemetry.update();
//            Thread.sleep(1000);
//        } else {
//
//            telemetry.addData("Shipping Hub Level", area);
//            telemetry.update();
//            Thread.sleep(1000);
//        }







//
//       turnClockWiseByTime(1000,-1);
//       driveForwardByTime(2000, 1);
    }
}


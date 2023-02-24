package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Camera_Array;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;


@Autonomous(name = "CAMS TEST")
public class Camera_TEST extends LinearOpMode {
        //RED + RIGHT
            //66,
    /**
     * Amount of time elapsed
     */
    private final ElapsedTime runtime = new ElapsedTime();
    private final RobotHardware_RB2_ robot = new RobotHardware_RB2_();
    private final Camera_Array cameras = new Camera_Array(telemetry);
    //private  SampleMecanumDrive driveTrain = null;
    private final Turret turret = new Turret();
    Thread telemetryHandler = new Thread(){
        @Override
        public void run() {
            while (opModeIsActive()) {
                telemetry.addData("Runtime(s): ", runtime.seconds());
                telemetry.addData("Yellow", cameras.getYellowLocation());
                telemetry.addData("turretpos", turret.getTurretPosition());
                telemetry.addData("cam1 -- position", cameras.yellowPos(1));
                telemetry.addData("cam2 -- position", cameras.yellowPos(2));
                telemetry.addData("cam 1 -- area", cameras.yellowArea(1));
                telemetry.addData("cam2 -- area", cameras.yellowArea(2));
                telemetry.addData("leftDist", robot.getLeftDistance());
                telemetry.addData("rightDist", robot.getRightDistance());
                telemetry.addData("movement", cameras.calculateMovement());
                telemetry.addData("focusCam1:", cameras.cam1Focus());

                //turret.lockOn(cameras.calculateMovement());
                turret.setTurretPosition(turret.getTurretPosition() + (gamepad1.right_stick_button? cameras.calculateMovement() : 0));
                if(gamepad1.a) {
                    FtcDashboard.getInstance().startCameraStream(cameras.getCamera(1), 0);
                }else {
                    FtcDashboard.getInstance().startCameraStream(cameras.getCamera(2), 0);
                }
                telemetry.update();
            }
        }
    };


    @Override
    public void runOpMode() {
        //driveTrain = new SampleMecanumDrive(hardwareMap);
        runtime.reset();
        telemetry.addData("Status", "Initializing");
        telemetry.update();
        robot.init(hardwareMap, telemetry);
        telemetry.addData("robotHardware", "done");
        telemetry.update();
        cameras.init(hardwareMap);
        telemetry.addData("cameras", "done");
        telemetry.update();
        turret.init(hardwareMap);
        while(!cameras.initialized() && runtime.seconds() < 3) {
            telemetry.addData("camera_initialized:", false);
            telemetry.update();
        }
        telemetry.addData("camera_initialized:", true);
        telemetry.update();
        while(!isStarted() && !isStopRequested()){
            cameras.run(gamepad2);
            telemetry.addData("waiting to start:", "...");
            telemetry.update();
            sleep(50);
        }
        //right red

        telemetry.addData("started:", true);
        telemetry.update();
        telemetryHandler.start();
        telemetry.update();
        robot.setSlidePosition(Constants.LOW_POSITION);
        while(opModeIsActive()){}

    }
}


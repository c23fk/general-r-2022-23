package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Camera_Array;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;
import org.firstinspires.ftc.teamcode.opencv.SignalColor;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name = "TestAuto", preselectTeleOp = "Layer Cake(1Driver)")
public class TestAuto_new extends LinearOpMode {
        //RED + RIGHT
            //66,
    /**
     * Amount of time elapsed
     */
    private final ElapsedTime runtime = new ElapsedTime();
    private final RobotHardware_RB2_ robot = new RobotHardware_RB2_();
    private final Camera_Array cameras = new Camera_Array(telemetry);

    private  SampleMecanumDrive driveTrain = null;
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
//                if(gamepad1.right_bumper) {
//                    turret.setAutoAdjust(true);
//                } else {
//                    turret.setAutoAdjust(false);
//                }
                turret.lockOn(cameras.calculateMovement());
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
        driveTrain = new SampleMecanumDrive(hardwareMap);
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
        int level;
        driveTrain.setPoseEstimate(new Pose2d(new Vector2d(65- robot.getRightDistance(),-66),Math.PI/2));
        TrajectorySequence initialSpline =driveTrain.trajectorySequenceBuilder(driveTrain.getPoseEstimate())
                .splineTo(new Vector2d(36,-36),Math.PI/2)
                .splineTo(new Vector2d(42,-13.5),0)
                .setReversed(true)
                .addTemporalMarker(() -> {
                    turret.setTurretPosition(0.56);
                    turret.setAutoAdjust(true);
                    robot.setSlidePosition(Constants.HIGH_POSITION);
                    robot.setWristPosition(Constants.WRIST_UP);
                })
                .splineTo(new Vector2d(24,-15),Math.PI)
                .build();

        TrajectorySequence pickupSpline = driveTrain.trajectorySequenceBuilder(initialSpline.end())
                .splineTo(new Vector2d(60,-14),0)
                .addTemporalMarker(() -> {
                    robot.setClawPosition(Constants.CLAW_CLOSED);
                    robot.setWristPosition(Constants.WRIST_UP);
                })
                .waitSeconds(0.25)
                .setReversed(true)
                .addTemporalMarker(() -> {
                    turret.setTurretPosition(0.56);
                    turret.setAutoAdjust(true);
                    robot.setSlidePosition(Constants.HIGH_POSITION);
                    robot.setWristPosition(Constants.WRIST_UP);
                })
                .splineTo(new Vector2d(24,-13.5),Math.PI)
                .build();
        telemetry.addData("path generation", "done");
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
        level = cameras.getTag();
        //waitForStart();
        int parkingLocation;
        switch (level) {
            case 1:
                parkingLocation = -1;
                break;
            case 2:
                parkingLocation = 0;
                break;
            default:
                parkingLocation = 1;
                break;
        }
        telemetryHandler.start();
        telemetry.addData("Status", "running...");
        telemetry.addData("Version", "1.0");
        telemetry.update();
        robot.setSlidePosition(Constants.LOW_POSITION);
        driveTrain.followTrajectorySequence(initialSpline);
        sleep(2000);
        robot.setWristPosition(Constants.WRIST_DOWN);
        sleep(250);
        robot.setClawPosition(Constants.CLAW_OPEN);
        turret.setAutoAdjust(false);
        turret.setTurretPosition(0.5);
        sleep(100);
        robot.setSlidePosition(130*5);
        driveTrain.followTrajectorySequence(pickupSpline);
        sleep(2000);
        robot.setWristPosition(Constants.WRIST_DOWN);
        robot.setClawPosition(Constants.CLAW_OPEN);
        sleep(250);
        turret.setAutoAdjust(false);
        turret.setTurretPosition(0.5);
        sleep(100);
        robot.setSlidePosition(130*4);
        while(opModeIsActive()){}
        //robot.forwardDrive(0.5,2000,1);
    }
}


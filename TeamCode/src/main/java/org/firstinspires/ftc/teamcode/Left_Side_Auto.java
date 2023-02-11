package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Camera_Array;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name = "Left_Side_Auto")
public class Left_Side_Auto extends LinearOpMode {
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
//        int level;
//        driveTrain.setPoseEstimate(new Pose2d(new Vector2d(-65+ robot.getLeftDistance(),-66),Math.PI/2));
//        TrajectorySequence initialSpline =driveTrain.trajectorySequenceBuilder(driveTrain.getPoseEstimate())
//                .splineTo(new Vector2d(-36,-36),Math.PI/2)
//                .splineTo(new Vector2d(-42,-13.5),Math.PI)
//                .setReversed(true)
//                .addTemporalMarker(() -> {
//                    turret.setTurretPosition(0.45);
//                    turret.setAutoAdjust(true);
//                    robot.setSlidePosition(Constants.HIGH_POSITION);
//                    robot.setWristPosition(Constants.WRIST_UP);
//                })
//                .splineTo(new Vector2d(-24,-14),0)
//                .build();
//        TrajectorySequence park1 = driveTrain.trajectorySequenceBuilder(new Pose2d(new Vector2d(24,-14),0))
//                .setReversed(true)
//                .splineTo(new Vector2d(12,-12),Math.PI)
//                .turn(Math.PI/2)
//                .build();
//        TrajectorySequence park2 = driveTrain.trajectorySequenceBuilder(new Pose2d(new Vector2d(24,-14),0))
//                .splineTo(new Vector2d(36,-12),0)
//                .turn(Math.PI/2)
//                .build();
//        TrajectorySequence park3 = driveTrain.trajectorySequenceBuilder(new Pose2d(new Vector2d(24,-14),0))
//                .splineTo(new Vector2d(60,-12),0)
//                .turn(Math.PI/2)
//                .build();
//        telemetry.addData("path generation", "done");
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
        telemetry.addData("Status", "running...");
        telemetry.addData("Version", "1.0");
        telemetry.update();
        robot.setSlidePosition(Constants.LOW_POSITION);
        //driveTrain.followTrajectorySequence(initialSpline);
        robot.forwardDrive(0.5,2450,3);
        turret.setTurretPosition(0.5);
        turret.setAutoAdjust(true);
        robot.setSlidePosition(Constants.HIGH_POSITION);
        robot.setWristPosition(Constants.WRIST_UP);
        robot.strafeRight(0.25,500,3);
        sleep(4000);
        robot.setWristPosition(Constants.WRIST_DOWN);
        sleep(500);
        robot.setClawPosition(Constants.CLAW_OPEN);
        sleep(500);
        turret.setAutoAdjust(false);
        turret.setTurretPosition(0.5);
        robot.setWristPosition(Constants.WRIST_UP);
        robot.setClawPosition(Constants.CLAW_OPEN);
        sleep(500);
        switch (cameras.getTag()) {
            case 1:
                robot.strafeRight(0.5,750,1);
                break;
            case 3:
                robot.strafeRight(0.5,-1750,3);
                break;
            default:
                robot.strafeRight(0.5,-750,1);
                break;
        }
        robot.setSlidePosition(0);
        robot.forwardDrive(0.25,-1000,1);
        sleep(1000);
    }
}


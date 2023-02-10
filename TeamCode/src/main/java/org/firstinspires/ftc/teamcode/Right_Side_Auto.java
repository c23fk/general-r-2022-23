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


@Autonomous(name = "Right_Side_Auto")
public class Right_Side_Auto extends LinearOpMode {
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
        int level;
//        driveTrain.setPoseEstimate(new Pose2d(new Vector2d(36,-66),Math.PI/2));
//        TrajectorySequence initialSpline =driveTrain.trajectorySequenceBuilder(driveTrain.getPoseEstimate())
//                .splineTo(new Vector2d(36,-18),Math.PI/2)
//                .addTemporalMarker(() -> {
//                    turret.setTurretPosition(0.5);
//                    turret.setAutoAdjust(true);
//                    robot.setSlidePosition(Constants.HIGH_POSITION);
//                    robot.setWristPosition(Constants.WRIST_UP);
//                })
//                .splineToConstantHeading(new Vector2d(24,-15),Math.PI/2)
//                .splineToConstantHeading(new Vector2d(20,-18),Math.PI/2)
//                .build();
//        TrajectorySequence park1 = driveTrain.trajectorySequenceBuilder(initialSpline.end())
//                .splineToConstantHeading(new Vector2d(12,-18),Math.PI/2)
//                .build();
//        TrajectorySequence park2 = driveTrain.trajectorySequenceBuilder(initialSpline.end())
//                .splineToConstantHeading(new Vector2d(36,-18),Math.PI/2)
//                .build();
//        TrajectorySequence park3 = driveTrain.trajectorySequenceBuilder(initialSpline.end())
//                .splineToConstantHeading(new Vector2d(57.5,-18),Math.PI/2)
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
        //right red

        telemetry.addData("started:", true);
        telemetry.update();
//        TrajectorySequence parkingSpline;

        telemetryHandler.start();
        telemetry.addData("Status", "running...");
        telemetry.addData("Version", "1.0");
        telemetry.update();
        robot.setSlidePosition(Constants.LOW_POSITION);
        //driveTrain.followTrajectorySequence(initialSpline);
        robot.forwardDrive(0.5,2300,3);
        turret.setTurretPosition(0.5);
        turret.setAutoAdjust(true);
        robot.setSlidePosition(Constants.HIGH_POSITION);
        robot.setWristPosition(Constants.WRIST_UP);
        robot.strafeRight(0.25,-750,3);
        sleep(5000);
        robot.setWristPosition(Constants.WRIST_DOWN);
        sleep(500);
        robot.setClawPosition(Constants.CLAW_OPEN);
        sleep(500);
        turret.setAutoAdjust(false);
        turret.setTurretPosition(0.5);
        robot.setWristPosition(Constants.WRIST_UP);
        robot.setClawPosition(Constants.CLAW_OPEN);
        sleep(500);
        level = cameras.getTag();
        switch (level) {
            case 1:
                robot.strafeRight(0.5,-750,1);
                break;
            case 3:
                robot.strafeRight(0.5,1250,1);
                break;
            default:
                robot.strafeRight(0.5,500,1);
                break;
        }
        robot.setSlidePosition(0);
        robot.forwardDrive(0.25,-1000,1);
        sleep(1000);

    }
}


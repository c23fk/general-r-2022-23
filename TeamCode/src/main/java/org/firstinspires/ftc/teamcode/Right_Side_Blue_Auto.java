package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Camera_Array;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;


@Autonomous(name = "Right_Side_Blue_Auto")
public class Right_Side_Blue_Auto extends LinearOpMode {
    //RED + RIGHT
    //66,
    /**
     * Amount of time elapsed
     */
    private final ElapsedTime runtime = new ElapsedTime();
    private String step = "init";
    private final RobotHardware_RB2_ robot = new RobotHardware_RB2_();
    private final Camera_Array cameras = new Camera_Array(telemetry);
    //private  SampleMecanumDrive driveTrain = null;
    private final Turret turret = new Turret();
    Thread telemetryHandler = new Thread(){
        @Override
        public void run() {
            while (opModeIsActive()) {
                telemetry.addData("Runtime(s): ", runtime.seconds());
                telemetry.addData("Version: ", "12");
                telemetry.addData("Yellow", cameras.getYellowLocation());
                telemetry.addData("turretpos", turret.getTurretPosition());
                telemetry.addData("cam1 -- position", cameras.yellowPos(1));
                telemetry.addData("cam2 -- position", cameras.yellowPos(2));
                telemetry.addData("cam 1 -- area", cameras.yellowArea(1));
                telemetry.addData("cam2 -- area", cameras.yellowArea(2));
                telemetry.addData("backDist", robot.getBackDist());
                telemetry.addData("frontDist", robot.getFrontDist());
                telemetry.addData("movement", cameras.calculateMovement());
                telemetry.addData("focusCam1:", cameras.cam1Focus());
                telemetry.addData("step:", step);
                telemetry.addData("angle:", robot.getAngle());

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
        //init
        runtime.reset();
        telemetry.addData("Status", "Initializing");
        telemetry.update();
        //hardware init
        robot.init(hardwareMap, telemetry);
        telemetry.addData("robotHardware", "done");
        telemetry.update();
        //cameras init
        cameras.init(hardwareMap);
        telemetry.addData("cameras", "done");
        telemetry.update();
        //turret init
        turret.init(hardwareMap);
        int level;
        //wait for match to start
        while(!isStarted() && !isStopRequested()){
            cameras.run(gamepad2);
            telemetry.addData("waiting to start:", "...");
            telemetry.update();
            sleep(50);
        }
        telemetryHandler.start();
        robot.setSlidePosition(Constants.LOW_POSITION);
        step = "going to junction 1";
        robot.forwardDrive(0.7,2400,3);
        robot.setSlidePosition(Constants.HIGH_POSITION);
        robot.setWristPosition(Constants.WRIST_UP);
        robot.strafeRight(0.25,-500,3);
        turret.setAutoAdjust(true);
        step = "placing first cone";
        sleep(3000);
        robot.setWristPosition(Constants.WRIST_DOWN);
        sleep(500);
        robot.setClawPosition(Constants.CLAW_OPEN);
        step = "reset for next cone";
        turret.setAutoAdjust(false);
        turret.setTurretPosition(0.5);
        //shift back a bit
        robot.setWristPosition(Constants.WRIST_DOWN);
        robot.setSlidePosition(550);
        robot.setClawPosition(Constants.CLAW_OPEN);
        step = "going to stack 1";
        robot.rotateToZero(-Math.PI/2,2);
        robot.strafeRight(0.25, 100,1);
        //move to junction
        int returnDistance = robot.forwardDrive(0.5, 1500,4,12)+100;
        robot.setClawPosition(Constants.CLAW_CLOSED);
        sleep(750);
        robot.setSlidePosition(Constants.LOW_POSITION);
        sleep(500);
        robot.setWristPosition(Constants.WRIST_UP);
        turret.setTurretPosition(Constants.TURRET_LEFT);
        step = "going back to junction";
        new Thread(){
            @Override
            public void run() {
                try {
                    sleep(1000);
                    robot.setSlidePosition(Constants.HIGH_POSITION);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        }.start();
        robot.forwardDrive(0.5,-returnDistance,3);
        turret.setAutoAdjust(true);
        step = "placing";
        robot.strafeRight(0.2,-100,1);
        sleep(3000);
        robot.setWristPosition(Constants.WRIST_DOWN);
        sleep(500);
        robot.setClawPosition(Constants.CLAW_OPEN);
        turret.setAutoAdjust(false);
        turret.setTurretPosition(0.5);
        robot.setWristPosition(Constants.WRIST_UP);
        robot.setClawPosition(Constants.CLAW_OPEN);
        step = "parking";
        robot.rotateToZero(0,2);
        level = cameras.getTag();
        switch (level) {
            case 1:
                robot.strafeRight(0.5,-750,1);
                break;
            case 3:
                robot.strafeRight(0.5,1750,3);
                break;
            default:
                robot.strafeRight(0.5,750,1);
                break;
        }
        robot.setSlidePosition(0);
        sleep(1000);

    }
}

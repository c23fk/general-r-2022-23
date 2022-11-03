package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.mechanisms.Camera;
import org.firstinspires.ftc.teamcode.opencv.SignalColor;


@Autonomous(name = "Right_Side_Blue")
public class RightBlueAuto extends LinearOpMode {

    /**
     * Amount of time elapsed
     */
    private ElapsedTime runtime = new ElapsedTime();
    private RobotHardware robot = new RobotHardware();
    private Camera camera = new Camera(telemetry);
    private int parkingLocation;
    Thread telemetryHandler = new Thread(){
        @Override
        public void run() {
            while (opModeIsActive()) {
                telemetry.addData("Runtime(s): ", runtime.seconds());
                telemetry.update();
            }
        }
    };


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();
        robot.init(hardwareMap, telemetry);
        camera.init(hardwareMap);
        SignalColor level;
        while(!camera.initialized()) {
            telemetry.addData("camera_initialized:", false);
            telemetry.update();
        }
        while(!isStarted() && !isStopRequested()){
            camera.run(gamepad2);
        }
        level = camera.getColor();
        waitForStart();
        switch (level) {
            case PURPLE:
                parkingLocation = -1;
                break;
            case GREEN:
                parkingLocation = 1;
                break;
            default:
                parkingLocation = 0;
                break;
        }
        telemetryHandler.start();
        robot.forwardDrive(0.75,875,2);
        robot.setSlidePosition(Constants.MID_POSITION);
        robot.rotateToZero(0,1);
        robot.strafeRight(-0.1);
        while(robot.getBackDistance()>30&&opModeIsActive()){}
        robot.stopDrive();
        robot.strafeRight(0.3, -50, 1);
        sleep(1500);
        robot.setWristPosition(Constants.WRIST_DOWN);
        sleep(1000);
        robot.setClawPosition(Constants.CLAW_OPEN);
        sleep(500);
        robot.setWristPosition(Constants.WRIST_UP);
        robot.setSlidePosition(0);
        sleep(1000);
        robot.strafeRight(0.5,350,1);
        robot.strafeRight(0.5,(parkingLocation*1000)+200,3);
        sleep(1000);
        robot.rotateToZero(0,1);
        runtime.reset();
    }


    public static void resetEncoder(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}


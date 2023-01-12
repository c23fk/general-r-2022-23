package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Camera_Array;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;
import org.firstinspires.ftc.teamcode.opencv.SignalColor;


@Autonomous(name = "TestAuto", preselectTeleOp = "Layer Cake(1Driver)")
public class TestAuto_new extends LinearOpMode {

    /**
     * Amount of time elapsed
     */
    private final ElapsedTime runtime = new ElapsedTime();
    private final RobotHardware_RB2_ robot = new RobotHardware_RB2_();
    private final Camera_Array cameras = new Camera_Array(telemetry);
    private final Turret turret = new Turret();
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
        cameras.init(hardwareMap);
        turret.init(hardwareMap);
        SignalColor level;
        while(!cameras.initialized()) {
            telemetry.addData("camera_initialized:", false);
            telemetry.update();
        }
        while(!isStarted() && !isStopRequested()){
            cameras.run(gamepad2);
        }
        level = cameras.getColor();
        waitForStart();
        int parkingLocation;
        switch (level) {
            case PURPLE:
                parkingLocation = -1;
                break;
            case ORANGE:
                parkingLocation = 0;
                break;
            default:
                parkingLocation = 1;
                break;
        }
        //telemetryHandler.start();
        while(opModeIsActive()) {
            robot.setSlidePosition(Constants.LOW_POSITION);
            telemetry.addData("Yellow", cameras.getYellowLocation());
            telemetry.addData("turretpos", turret.getTurretPosition());
            double offset =  cameras.getYellowLocation()/Constants.CAM_WIDTH - 0.3125;
            double movement = offset * 0.01;
            telemetry.addData("offset", offset);
            telemetry.addData("movement", movement);
            telemetry.addData("focusCam1:", cameras.cam1Focus());
            if(gamepad1.right_bumper) {
                turret.setTurretPosition(turret.getTurretPosition() + movement);
                sleep(100);
            }
            telemetry.update();
        }
    }
}


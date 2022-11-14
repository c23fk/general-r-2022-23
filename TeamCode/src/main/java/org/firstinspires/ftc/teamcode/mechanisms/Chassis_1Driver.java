package org.firstinspires.ftc.teamcode.mechanisms;

import static java.lang.Math.PI;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class Chassis_1Driver implements Mechanism{
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private BNO055IMU imu = null;

    public Chassis_1Driver() {}

    @Override
    public void init(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");
        //imu
        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void run(Gamepad gamepad) {
        //Get the positions of the left stick in terms of x and y
        //Invert y because of the input from the controller
        double stickX = Math.abs(gamepad.left_stick_x) < Constants.STICK_THRESH ? 0 : gamepad.left_stick_x;
        double stickY = Math.abs(gamepad.left_stick_y) < Constants.STICK_THRESH ? 0 : -gamepad.left_stick_y;
        //get the direction from the IMU
        double angle = imu.getAngularOrientation().firstAngle;
        //rotate the positions to prep for wheel powers
        double rotatedX = (stickX * Math.cos(PI / 4)) - (stickY * Math.sin(PI / 4));
        double rotatedY = (stickY * Math.cos(PI / 4)) + (stickX * Math.sin(PI / 4));
        //determine how much the robot should turn
        double rotation = (gamepad.left_trigger - gamepad.right_trigger) * Constants.ROTATION_SENSITIVITY;
        //test if the robot should move
        double stickPower = Math.sqrt(rotatedX * rotatedX + rotatedY * rotatedY);
        boolean areTriggersDown = Math.abs(rotation) > 0;
        boolean areSticksMoved = stickPower > 0;
        if (areSticksMoved || areTriggersDown) {
            //add the rotation to the powers of the wheels
            double motorMax = Math.max(Math.abs(rotatedX)+rotation, Math.abs(rotatedY)+rotation);
            double proportion = Math.max(1, motorMax);
            double num = (1 / proportion)* stickPower;
            double flPower = num * -rotatedY + rotation;
            double brPower = num * rotatedY/proportion + rotation;
            double frPower = num * -rotatedX/proportion + rotation;
            double blPower = num * rotatedX/proportion + rotation;
            //keep the powers proportional and within a range of -1 to 1

            frontLeft.setPower(flPower / proportion);
            backRight.setPower(brPower / proportion);
            frontRight.setPower(frPower / proportion);
            backLeft.setPower(blPower / proportion);
        } else {
            stopDrive();
        }
    }

    public void stopDrive() {
        frontLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
    }

    public double getAngle(){
        return imu.getAngularOrientation().firstAngle;
    }

    public double getFrontLeftPosition() {
        return frontLeft.getCurrentPosition();
    }

    public double getFrontRightPosition() {
        return frontRight.getCurrentPosition();
    }

    public double getBackLeftPosition() {
        return backLeft.getCurrentPosition();
    }

    public double getBackRightPosition() {
        return backRight.getCurrentPosition();
    }
}

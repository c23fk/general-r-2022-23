package org.firstinspires.ftc.teamcode.mechanisms;

import static java.lang.Math.PI;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;

public class Chasis implements Mechanism{
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private BNO055IMU imu = null;

    public Chasis() {}

    @Override
    public void init(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");
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
        double rotatedX = (stickX * Math.cos(PI / 4 - angle)) - (stickY * Math.sin(PI / 4 - angle));
        double rotatedY = (stickY * Math.cos(PI / 4 - angle)) + (stickX * Math.sin(PI / 4 - angle));
        if(rotatedX == 0 && rotatedY == 0){
            stickX = Math.abs(gamepad.right_stick_x) < Constants.STICK_THRESH ? 0 : gamepad.right_stick_x;
            stickY = Math.abs(gamepad.right_stick_y) < Constants.STICK_THRESH ? 0 : -gamepad.right_stick_y;
            rotatedX = (stickX * Math.cos(PI / 4)) - (stickY * Math.sin(PI / 4));
            rotatedY = (stickY * Math.cos(PI / 4)) + (stickX * Math.sin(PI / 4));
        }
        //determine how much the robot should turn
        double rotation = gamepad.left_trigger * Constants.ROTATION_SENSITIVITY - gamepad.right_trigger * Constants.ROTATION_SENSITIVITY;
        //test if the robot should move
        boolean areTriggersDown = Math.abs(rotation) > Constants.STICK_THRESH;
        boolean areSticksMoved = Math.sqrt((rotatedX * rotatedX) + (rotatedY * rotatedY)) > Constants.STICK_THRESH;
        if (areSticksMoved || areTriggersDown) {
            //add the rotation to the powers of the wheels
            double flPower = -rotatedY + rotation;
            double brPower = rotatedY + rotation;
            double frPower = -rotatedX + rotation;
            double blPower = rotatedX + rotation;
            //keep the powers proportional and within a range of -1 to 1
            double motorMax = Math.max(Math.max(Math.abs(flPower), Math.abs(brPower)), Math.max(Math.abs(frPower), Math.abs(blPower)));
            double proportion = Math.max(1, motorMax);
            frontLeft.setPower(flPower / proportion);
            backRight.setPower(brPower / proportion);
            frontRight.setPower(frPower / proportion);
            backLeft.setPower(blPower / proportion);
        } else {
            stopDrive();
        }
        if (gamepad.y) {
            rotateToZero(0);
        }
    }

    public void stopDrive() {
        frontLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
    }

    private void rotateToZero(double angle) {
        //use a PID control loop to zero
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        double k_p = Math.PI/8;
        double k_i = 0;
        double k_d = 2;
        double current_error = imu.getAngularOrientation().firstAngle - angle;
        double previous_error = current_error;
        double previous_time = 0;
        double current_time = 0;
        double max_i = 0.1;
        //while(timer.seconds() < 5){
        while (Math.abs(imu.getAngularOrientation().firstAngle - angle) > Constants.TOLERANCE) {
            current_time = timer.milliseconds();
            current_error = angle - imu.getAngularOrientation().firstAngle;
            double p = k_p * current_error;
            double i = k_i * (current_error * (current_time - previous_time));
            i = Range.clip(i, -max_i, max_i);
            double d = k_d * ((current_error - previous_error) / (current_time - previous_time));
            double power = p + i + d;
            frontLeft.setPower(power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);
            previous_error = current_error;
            previous_time = current_time;
        }

        stopDrive();
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

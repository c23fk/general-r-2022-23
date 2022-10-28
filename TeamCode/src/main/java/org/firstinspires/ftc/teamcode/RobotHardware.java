/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opencv.ColorPipeline;
import org.firstinspires.ftc.teamcode.opencv.SignalColor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
//import org.firstinspires.ftc.teamcode.vision.CubePipeline;
//import org.firstinspires.ftc.teamcode.vision.ShippingElementPipeline;
//import org.opencv.core.Point;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;

public class RobotHardware  {
    /* Public OpMode members. */
    public DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor slides = null;
    private Servo wrist = null;
    private Servo claw = null;
    private DistanceSensor distLeft = null;
    private DistanceSensor distRight = null;
    private DistanceSensor distBack = null;
    private BNO055IMU imu = null;
    private OpenCvWebcam webcam;
    private ColorPipeline pipeline;
    private SignalColor color;
    private boolean sensorFail = false;
    /* local OpMode members. */
    HardwareMap hardwareMap = null;
    Telemetry telemetry;


    /* Constructor */
    public RobotHardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap,Telemetry telemetry) {
        // Save reference to Hardware map
        hardwareMap = ahwMap;
        this.telemetry = telemetry;


        // Define and initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backRight = hardwareMap.get(DcMotor.class, "br");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        //spin = hardwareMap.get(DcMotor.class, "spin");
        slides = hardwareMap.get(DcMotor.class, "slides");
        //intake = hardwareMap.get(DcMotor.class, "nom");
        //set the direction of each motor
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        //spin.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setDirection(DcMotorSimple.Direction.FORWARD);
//        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        //set zero power behaviors for each motor
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //spin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        //init slides
        claw.setPosition(Constants.CLAW_CLOSED);
        wrist.setPosition(0.2);
        // Set all motors to zero power
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        slides.setPower(0);


        // Reset all encoders and set the motors to run using the encoders
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        initSlides();
        //define and initialize sensors
        distLeft = hardwareMap.get(DistanceSensor.class, "distLeft");
        distRight = hardwareMap.get(DistanceSensor.class, "distRight");
        distBack = hardwareMap.get(DistanceSensor.class, "distBack");
        //define and initialize imu
        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);
        //initialize the camera
        telemetry.addData("Camera status:", "waiting");
        telemetry.update();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        pipeline = new ColorPipeline(telemetry);
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(Constants.CAM_WIDTH, Constants.CAM_HEIGHT, OpenCvCameraRotation.SIDEWAYS_LEFT);
                //telemetry.addData("Camera status:", "initialized");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                // This will be called if the camera could not be opened
            }
        });
        while(pipeline.getColor() == SignalColor.UNSET){
            telemetry.addData("camera ready?", "false");
            telemetry.addData("pipeline chosen:", "colorPipeline");
            telemetry.update();
        }
        color = pipeline.getColor();
        telemetry.addData("camera ready?", "true");
        telemetry.addData("pipeline chosen", "Shipping");
        telemetry.update();
    }

    public SignalColor getColor(){
        return pipeline.getColor();
    }

    public void initSlides() {
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setTargetPosition(Constants.INTAKE_POSITION);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(Constants.SLIDE_POWER);
    }

    public void setSlidePosition(int pos) {
        slides.setTargetPosition(pos);
    }

    public double getLeftDistance() {
        return distLeft.getDistance(DistanceUnit.CM);
    }

    public double getRightDistance() {
        return distRight.getDistance(DistanceUnit.CM);
    }

    public double getBackDistance() {
        return distBack.getDistance(DistanceUnit.CM);
    }

    //rotation
    public void rotateLeft(double power, double timeout) {
        ElapsedTime timer = new ElapsedTime();
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        //noinspection StatementWithEmptyBody
        while (timer.seconds() < timeout) {
        }
        stopDrive();
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void rotateLeft(double power, int position, double timeout) {
        //TIMER :)
        ElapsedTime timer = new ElapsedTime();
        //reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //set target positions
        frontLeft.setTargetPosition(position);
        frontRight.setTargetPosition(position);
        backLeft.setTargetPosition(position);
        backRight.setTargetPosition(position);
        //set to run to position
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //set powers
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        //noinspection StatementWithEmptyBody
        while ((frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()) && timer.seconds() < timeout) {
        }
        stopDrive();
    }

    //drive forward
    public void forwardDrive(double power) {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(-power);
        backRight.setPower(power);
    }
    public void forwardDrive(double power, int position, double timeout) {
        //TIMER :)
        ElapsedTime timer = new ElapsedTime();
        //reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //set target positions
        frontLeft.setTargetPosition(-position);
        frontRight.setTargetPosition(position);
        backLeft.setTargetPosition(-position);
        backRight.setTargetPosition(position);
        //set to run to position
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //set powers
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        //noinspection StatementWithEmptyBody
        while ((frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()) && timer.seconds() < timeout) {
        }
        stopDrive();
    }

    //strafe right
    public void strafeRight(double power) {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setPower(-power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    public void strafeLeft(double power) {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(-power);
        backRight.setPower(-power);
    }
    public void strafeRight(double power, int position, double timeout) {
        //TIMER :)
        ElapsedTime timer = new ElapsedTime();
        //reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //set target positions
        frontLeft.setTargetPosition(-position);
        frontRight.setTargetPosition(-position);
        backLeft.setTargetPosition(position);
        backRight.setTargetPosition(position);
        //set to run to position
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //set powers
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        //noinspection StatementWithEmptyBody
        while ((frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()) && timer.seconds() < timeout) {
        }
        stopDrive();
    }


    public void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public double getAngle(){
        return imu.getAngularOrientation().firstAngle;
    }

    public void openClaw(){
        claw.setPosition(Constants.CLAW_OPEN);
    }

    public void closeClaw(){
        claw.setPosition(Constants.CLAW_CLOSED);
    }

    public void wristUp(){
        wrist.setPosition(0.25);
    }

    public void wristDown(){
        wrist.setPosition(0.5);
    }




    public void driveByAngleSensor(double angle, DistanceSensor sensor, double toDistance, double timeout){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        double newAngle = Math.toRadians(angle + 90);
        double current_error = sensor.getDistance(DistanceUnit.CM) - toDistance;
        double previous_error = current_error;
        double previous_time = 0;
        double current_time;
        double max_i = 0.1;
        double initialAngle = imu.getAngularOrientation().firstAngle;
        double k_p = 0.01;
        double k_i = 1;
        double k_d = 1;
        while(Math.abs(sensor.getDistance(DistanceUnit.CM)-toDistance) > 1 && timer.seconds()<timeout){
            if(sensor.getDistance(DistanceUnit.CM)>800){
                sensorFail = true;
            }
            if(sensorFail){
                stopDrive();
                return;
            }
            current_time = timer.milliseconds();
            current_error = sensor.getDistance(DistanceUnit.CM)- toDistance;
            double p = k_p * current_error;
            double i = k_i * (current_error * (current_time - previous_time));
            i = Range.clip(i, -max_i, max_i);
            double d = k_d * ((current_error - previous_error) / (current_time - previous_time));
            double power = p + i + d;
            double startAngle = imu.getAngularOrientation().firstAngle;
            double rotatedX = (Math.cos(newAngle) * Math.cos(Math.PI / 4-startAngle)) - (Math.sin(newAngle) * Math.sin(Math.PI / 4-startAngle));
            double rotatedY = (Math.sin(newAngle) * Math.cos(Math.PI / 4-startAngle)) + (Math.cos(newAngle) * Math.sin(Math.PI / 4-startAngle));
            double angleError = initialAngle - startAngle;
            double rotation = angleError/(2*Math.PI);
            double flPower = -rotatedY + rotation;
            double brPower = rotatedY + rotation;
            double frPower = -rotatedX + rotation;
            double blPower = rotatedX + rotation;
            double proportion = Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)),Math.max(Math.abs(brPower), Math.abs(blPower)));
            flPower *= proportion;
            frPower *= proportion;
            brPower *= proportion;
            blPower *= proportion;
            frontLeft.setPower(power*flPower);
            frontRight.setPower(power*frPower);
            backLeft.setPower(power*blPower);
            backRight.setPower(power*brPower);
            previous_error = current_error;
            previous_time = current_time;
            telemetry.addData("angle error",angleError);
            telemetry.addData("angle correction",rotation);
            telemetry.addData("dist",sensor.getDistance(DistanceUnit.CM));
            telemetry.addData("error",current_error);
            telemetry.update();

        }
        stopDrive();
        telemetry.update();
    }

    public void driveByAngleEncoder(double angle, double distance, double targetRotation, double power, double timeout) {
        //timer
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        //get movement direction in rads
        double newAngle = Math.toRadians(angle + 90);
        //reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //get start angle
        double startAngle = imu.getAngularOrientation().firstAngle;
        //get components of original vector
        double xComponent = Math.cos(newAngle);
        double yComponent = Math.sin(newAngle);
        //rotate vector 45 degrees
        double rotatedX = (xComponent * Math.cos(Math.PI / 4-startAngle)) - (yComponent * Math.sin(Math.PI / 4-startAngle));
        double rotatedY = (yComponent * Math.cos(Math.PI / 4-startAngle)) + (xComponent * Math.sin(Math.PI / 4-startAngle));
        //get rotation angle in rads
        double targetRotationRad = Math.toRadians(targetRotation);
        //get needed rotation
        double rotation = targetRotationRad - startAngle;
        //get rotation between -360 and 360 degrees
        while (rotation > Math.PI * 2) {
            rotation -= Math.PI * 2;
        }
        while (rotation < -Math.PI * 2) {
            rotation += Math.PI * 2;
        }
        //make sure turn direction is correct
        if (rotation < -Math.PI) {
            rotation = Math.PI * 2 + rotation;
        }
        if (rotation > Math.PI) {
            rotation = -Math.PI * 2 + rotation;
        }
        //get the number of encoder counts for the target rotation
        double rotationInEncoderCounts = (rotation / (2 * Math.PI)) * Constants.FULL_SPIN;
        //setup target positions for each wheel
        double flTarget = (-rotatedY * distance) + rotationInEncoderCounts;
        double brTarget = (rotatedY * distance) + rotationInEncoderCounts;
        double frTarget = (-rotatedX * distance) + rotationInEncoderCounts;
        double blTarget = (rotatedX * distance) + rotationInEncoderCounts;
        //set target positions
        frontLeft.setTargetPosition((int) flTarget);
        backRight.setTargetPosition((int) brTarget);
        frontRight.setTargetPosition((int) frTarget);
        backLeft.setTargetPosition((int) blTarget);
        //prep motors
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //make powers less than 1
        double proportion = Math.max(Math.max(Math.abs(flTarget), Math.abs(brTarget)), Math.max(Math.abs(frTarget), Math.abs(blTarget)));
        //set the powers
        frontLeft.setPower(power * flTarget / proportion);
        frontRight.setPower(power * frTarget / proportion);
        backLeft.setPower(power * blTarget / proportion);
        backRight.setPower(power * brTarget / proportion);
        //wait until the motors finish or time expires
        //noinspection StatementWithEmptyBody
        while ((frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()) && timer.seconds() < timeout) {
        }
        //end the path
        //stopDrive();
    }

    public void driveByTime(double angle, double time, double targetRotation, double power, double timeout) {
        //timer
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        //get movement direction in rads
        double newAngle = Math.toRadians(angle + 90);
        //reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //get start angle
        double startAngle = imu.getAngularOrientation().firstAngle;
        //get components of original vector
        double xComponent = Math.cos(newAngle);
        double yComponent = Math.sin(newAngle);
        //rotate vector 45 degrees
        double rotatedX = (xComponent * Math.cos(Math.PI / 4-startAngle)) - (yComponent * Math.sin(Math.PI / 4-startAngle));
        double rotatedY = (yComponent * Math.cos(Math.PI / 4-startAngle)) + (xComponent * Math.sin(Math.PI / 4-startAngle));
        //get rotation angle in rads
        double targetRotationRad = Math.toRadians(targetRotation);
        //get needed rotation
        double rotation = targetRotationRad - startAngle;
        //get rotation between -360 and 360 degrees
        while (rotation > Math.PI * 2) {
            rotation -= Math.PI * 2;
        }
        while (rotation < -Math.PI * 2) {
            rotation += Math.PI * 2;
        }
        //make sure turn direction is correct
        if (rotation < -Math.PI) {
            rotation = Math.PI * 2 + rotation;
        }
        if (rotation > Math.PI) {
            rotation = -Math.PI * 2 + rotation;
        }
        //get the number of encoder counts for the target rotation
        double rotationInEncoderCounts = (rotation / (2 * Math.PI)) * Constants.FULL_SPIN;
        //setup target positions for each wheel
        double flTarget = (-rotatedY * time) + rotationInEncoderCounts;
        double brTarget = (rotatedY * time) + rotationInEncoderCounts;
        double frTarget = (-rotatedX * time) + rotationInEncoderCounts;
        double blTarget = (rotatedX * time) + rotationInEncoderCounts;
        //set target positions
        //make powers less than 1
        double proportion = Math.max(Math.max(Math.abs(flTarget), Math.abs(brTarget)), Math.max(Math.abs(frTarget), Math.abs(blTarget)));
        //set the powers
        frontLeft.setPower(power * flTarget / proportion);
        frontRight.setPower(power * frTarget / proportion);
        backLeft.setPower(power * blTarget / proportion);
        backRight.setPower(power * brTarget / proportion);
        //wait until the motors finish or time expires
        //noinspection StatementWithEmptyBody
        while ((frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()) && timer.seconds() < timeout) {
        }
        //end the path
        //stopDrive();
    }

}


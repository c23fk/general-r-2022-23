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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechanisms.Chassis_Robot2;

public class RobotHardware_RB2_ {
    /* Public OpMode members. */
    public Chassis_Robot2 chassis = new Chassis_Robot2();
    private DcMotor slides = null;
    private Servo wrist = null;
    private Servo claw = null;
    private BNO055IMU imu = null;
    private DistanceSensor leftDist = null;
    private DistanceSensor rightDist = null;
    private ColorSensor color = null;
    /* local OpMode members. */
    HardwareMap hardwareMap = null;
    Telemetry telemetry;


    /* Constructor */
    public RobotHardware_RB2_() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap,Telemetry telemetry) {
        // Save reference to Hardware map
        hardwareMap = ahwMap;
        this.telemetry = telemetry;

        chassis.init(hardwareMap);

        // Define and initialize motors
        leftDist = hardwareMap.get(DistanceSensor.class, "leftDist");
        rightDist = hardwareMap.get(DistanceSensor.class, "rightDist");
        color = hardwareMap.get(ColorSensor.class, "color");


        slides = hardwareMap.get(DcMotor.class, "slides");

        //set the direction of each motor
//
        //set zero power behaviors for each motor
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        //init slides
        claw.setPosition(Constants.CLAW_CLOSED);
        wrist.setPosition(Constants.WRIST_DOWN);
        slides.setPower(0);

        // Reset all encoders and set the motors to run using the encoders
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        initSlides();
        //define and initialize imu
        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);
        //initialize the camera
    }


    public void initSlides() {
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setTargetPosition(0);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(Constants.SLIDE_POWER);
    }

    public void setSlidePosition(int pos) {
        slides.setTargetPosition(pos);
    }

    public void setClawPosition(double pos){
        claw.setPosition(pos);
    }

    public void setWristPosition(double pos){
        wrist.setPosition(pos);
    }

    //rotation
    @Deprecated
    public void rotateLeft(double power, int position, double timeout) {
        chassis.rotateLeft(power,position,timeout);
    }

    //drive forward
    public void forwardDrive(double power, int position, double timeout) {
        chassis.forwardDrive(power,position,timeout);
    }

    public int forwardDrive(double power, int position, double timeout,double distance) {
        return chassis.forwardDrive(power,position,timeout,distance);
    }

    //strafe right
    public void strafeRight(double power, int position, double timeout) {
        chassis.strafeRight(power,position,timeout);
    }
    public void fr45(double power, int position, double timeout){
        chassis.fr45(power, position, timeout);
    }
    public void fl45(double power, int position, double timeout){
        chassis.fl45(power, position, timeout);
    }
    public void stopDrive() {
        chassis.stopDrive();
    }

    public double getAngle(){
        return imu.getAngularOrientation().firstAngle;
    }

    public void rotateToZero(double angle, double timeout) {
        chassis.rotateToZero(angle,timeout);
    }

    public double getLeftDistance(){
        return leftDist.getDistance(DistanceUnit.INCH);
    }

    public double getRightDistance(){
        return rightDist.getDistance(DistanceUnit.INCH);
    }

}


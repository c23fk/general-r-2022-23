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

import static java.lang.Math.PI;

import android.graphics.Camera;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that rus in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Layer Cake", group = "Iterative Opmode")

public class Iterative_Opmode_V_2 extends OpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor spin = null;
    private DcMotor slides = null;
    private DcMotor intake = null;
    private DistanceSensor distLeft = null;
    private DistanceSensor distRight = null;
    private DistanceSensor distBack = null;
    private DigitalChannel magSwitch = null;
    private BNO055IMU imu = null;
    private Servo wrist = null;
    private Servo claw = null;
    private int slidesTarget = Constants.INTAKE_POSITION;
    public double boxNum = 0.830;
    public double clawNum = 0.0;
    private double boxTest = 0.5;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backRight = hardwareMap.get(DcMotor.class, "br");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        spin = hardwareMap.get(DcMotor.class, "spin");
        slides = hardwareMap.get(DcMotor.class, "slides");
        intake = hardwareMap.get(DcMotor.class, "nom");
        //initialize the imu
        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        spin.setDirection(DcMotorSimple.Direction.FORWARD);
        slides.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        //set zero behaviors
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //reset encoders for all the motors
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //initialize distance sensors
        distLeft = hardwareMap.get(DistanceSensor.class, "distLeft");
        distRight = hardwareMap.get(DistanceSensor.class, "distRight");
        distBack = hardwareMap.get(DistanceSensor.class, "distBack");
        magSwitch = hardwareMap.get(DigitalChannel.class, "mag");
        magSwitch.setMode(DigitalChannel.Mode.INPUT);
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        //init slides
        //boxDoor.setPosition(Constants.BOX_CLOSED);
        //claw.setPosition(Constants.CLAW_OPEN);
        slides.setTargetPosition(slidesTarget);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(Constants.SLIDE_POWER);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //Get the positions of the left stick in terms of x and y
        //Invert y because of the input from the controller
        double stickX = Math.abs(gamepad1.left_stick_x) < Constants.STICK_THRESH ? 0 : gamepad1.left_stick_x;
        double stickY = Math.abs(gamepad1.left_stick_y) < Constants.STICK_THRESH ? 0 : -gamepad1.left_stick_y;
        //get the direction from the IMU
        double angle = imu.getAngularOrientation().firstAngle;
        //rotate the positions to prep for wheel powers
        double rotatedX = (stickX * Math.cos(PI / 4 - angle)) - (stickY * Math.sin(PI / 4 - angle));
        double rotatedY = (stickY * Math.cos(PI / 4 - angle)) + (stickX * Math.sin(PI / 4 - angle));
        //determine how much the robot should turn
        double rotation = gamepad1.left_trigger * Constants.ROTATION_SENSITIVITY - gamepad1.right_trigger * Constants.ROTATION_SENSITIVITY;
        //test if the robot should move
        boolean areTriggersDown = Math.abs(rotation) > Constants.STICK_THRESH;
        boolean areSticksMoved = Math.sqrt((stickX * stickX) + (stickY * stickY)) > Constants.STICK_THRESH;
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
        //duck spinner
        if (gamepad2.a) {
            spin.setPower(Constants.DUCK_POWER);
        } else if (gamepad2.b) {
            spin.setPower(-Constants.DUCK_POWER);
        } else {
            spin.setPower(0);
        }
        //slides
        if (gamepad2.dpad_up) {
            slidesTarget = Constants.HIGH_POSITION + 50;
            //boxDoor.setPosition(Constants.BOX_CLOSED);
        } else if (gamepad2.dpad_right) {
            slidesTarget = Constants.MID_POSITION;
            //boxDoor.setPosition(Constants.BOX_CLOSED);
        } else if (gamepad2.dpad_left) {
            slidesTarget = Constants.LOW_POSITION;
            //boxDoor.setPosition(Constants.BOX_CLOSED);
        } else if (gamepad2.dpad_down) {
            slidesTarget = Constants.INTAKE_POSITION;
            //boxDoor.setPosition(Constants.BOX_CLOSED);
        } else if (gamepad2.triangle) {
            slidesTarget = Constants.SHARED_POSITION;
            //boxDoor.setPosition(Constants.BOX_CLOSED);
        }
        //manual adjustments to slide positions
        slidesTarget += -gamepad2.right_stick_y * 25;
        slidesTarget = Range.clip(slidesTarget, -50, Constants.SLIDE_MAX);
        slides.setTargetPosition(slidesTarget);
        slides.setPower(Constants.SLIDE_POWER);
        //reset the zero position of the slides
        if (gamepad2.x) {
            slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
//        if (gamepad1.b) {
//            claw.setPosition(Constants.CLAW_CLOSED);
//        }
//        if (gamepad1.y) {
//            claw.setPosition(Constants.CLAW_OPEN);
//        }

        //intake and output
//        if (gamepad2.right_bumper && slides.getCurrentPosition() >= Constants.LOW_POSITION - 100) {
//            wrist.setPosition(Constants.BOX_OPEN);
//            intake.setPower(Constants.OUTPUT_POWER);
//        } else if (gamepad2.left_bumper) {
//            intake.setPower(Constants.INTAKE_POWER);
//        } else if (gamepad1.right_bumper) {
//            wrist.setPosition(Constants.BOX_OPEN);
//            intake.setPower(Constants.OUTPUT_POWER);
//        } else if (gamepad1.left_bumper) {
//            intake.setPower(Constants.INTAKE_POWER);
//        } else {
//            intake.setPower(0);
//            //boxDoor.setPosition(Constants.BOX_CLOSED);
//        }
        if (gamepad1.triangle) {
            rotateToZero(0);
        }
        //telemetry

//        if(gamepad1.right_stick_y > 0.1){
//            clawNum+=0.01;
//        }
//        if(gamepad1.right_stick_y < -0.1){
//            clawNum-=0.01;
//        }
        //boxDoor.setPosition(boxNum);
        //claw.setPosition(clawNum);
        telemetry.addData("claw position: ", clawNum);
        telemetry.addData("wrist position: ", wrist.getPosition());
        telemetry.addData("Slide Position: ", slides.getCurrentPosition());
        telemetry.addData("Distance on the left(cm): ", distLeft.getDistance(DistanceUnit.CM));
        telemetry.addData("Distance on the right(cm): ", distRight.getDistance(DistanceUnit.CM));
        telemetry.addData("Distance on the back(cm): ", distBack.getDistance(DistanceUnit.CM));
        telemetry.addData("FL: ", frontLeft.getCurrentPosition());
        telemetry.addData("FR: ", frontRight.getCurrentPosition());
        telemetry.addData("BL: ", backLeft.getCurrentPosition());
        telemetry.addData("BR: ", backRight.getCurrentPosition());
        telemetry.addData("Angle: ", imu.getAngularOrientation().firstAngle);
        telemetry.addData("RIGHT TRIGGER: ", gamepad1.right_trigger);
    }

    private void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */

    private void rotateToZero(double angle) {
        //use a PID control loop to
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
            current_time = runtime.milliseconds();
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

            telemetry.addData("status:", "zeroing direction");
            telemetry.addData("time(ms):", timer.milliseconds());
            telemetry.update();
        }

        stopDrive();
    }
    @Override
    public void stop() {
    }

}

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


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechanisms.Chassis_2Drivers;
import org.firstinspires.ftc.teamcode.mechanisms.Claw_2Drivers;

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

@TeleOp(name = "Layer Cake(2 Drivers)", group = "Iterative Opmode")
public class Iterative_Opmode_V_2_1 extends OpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor slides = null;
    private DistanceSensor distLeft = null;
    private DistanceSensor distRight = null;
    private DistanceSensor distBack = null;
    private final Claw_2Drivers claw = new Claw_2Drivers();
    private final Chassis_2Drivers chassis = new Chassis_2Drivers();
    private int slidesTarget = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        slides = hardwareMap.get(DcMotor.class, "slides");
        // Reverse the motor that runs backwards when connected directly to the battery
        slides.setDirection(DcMotorSimple.Direction.FORWARD);
        //set zero behaviors
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //reset encoders for all the motors
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //initialize distance sensors
        distLeft = hardwareMap.get(DistanceSensor.class, "distLeft");
        distRight = hardwareMap.get(DistanceSensor.class, "distRight");
        distBack = hardwareMap.get(DistanceSensor.class, "distBack");

        //init slides
        slides.setTargetPosition(slidesTarget);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(Constants.SLIDE_POWER);
        // Tell the driver that initialization is complete.

        claw.init(hardwareMap);
        chassis.init(hardwareMap);
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

        //slide presets
        if (gamepad2.dpad_up) {
            slidesTarget = Constants.SLIDE_MAX;
        } else if (gamepad2.dpad_right) {
            slidesTarget = Constants.MID_POSITION;
        } else if (gamepad2.dpad_left) {
            slidesTarget = Constants.LOW_POSITION;
        } else if (gamepad2.dpad_down) {
            slidesTarget = 0;
        }
        //manual adjustments to slide positions
        slidesTarget += -gamepad2.right_stick_y * 50;
        slidesTarget = Range.clip(slidesTarget, -50, Constants.SLIDE_MAX);
        //move the slides
        slides.setTargetPosition(slidesTarget);
        slides.setPower(Constants.SLIDE_POWER);
        //reset the zero position of the slides
        if (gamepad2.x) {
            slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        claw.run(gamepad2);
        chassis.run(gamepad1);

        telemetry.addData("wristPos: ", claw.getClawPosition());
        telemetry.addData("Slide Position: ", slides.getCurrentPosition());
        telemetry.addData("Distance on the left(cm): ", distLeft.getDistance(DistanceUnit.CM));
        telemetry.addData("Distance on the right(cm): ", distRight.getDistance(DistanceUnit.CM));
        telemetry.addData("Distance on the back(cm): ", distBack.getDistance(DistanceUnit.CM));
        telemetry.addData("FL: ", chassis.getFrontLeftPosition());
        telemetry.addData("FR: ", chassis.getFrontRightPosition());
        telemetry.addData("BL: ", chassis.getBackLeftPosition());
        telemetry.addData("BR: ", chassis.getBackRightPosition());
        telemetry.addData("Angle: ", chassis.getAngle());
    }



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

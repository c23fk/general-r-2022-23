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


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Camera_Array;
import org.firstinspires.ftc.teamcode.mechanisms.Chassis_Robot2;
import org.firstinspires.ftc.teamcode.mechanisms.Chassis_Robot2_1Driver;
import org.firstinspires.ftc.teamcode.mechanisms.Claw_1Driver;
import org.firstinspires.ftc.teamcode.mechanisms.Claw_2Drivers;
import org.firstinspires.ftc.teamcode.mechanisms.Slides;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;
import org.firstinspires.ftc.teamcode.mechanisms.Turret_1Driver;

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

@TeleOp(name = "Turret_Bot_1_driver", group = "Iterative Opmode")
public class teleop_1Driver extends OpMode {
    // Declare OpMode members.
    private boolean bliped  = false;
    private final ElapsedTime runtime = new ElapsedTime();
    private final Camera_Array cameras = new Camera_Array(telemetry);
    private final Chassis_Robot2_1Driver chassis = new Chassis_Robot2_1Driver();
    private final Claw_1Driver claw = new Claw_1Driver();
    private final Turret_1Driver turret = new Turret_1Driver();
    private final Slides slides = new Slides();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        chassis.init(hardwareMap);
        claw.init(hardwareMap);
        turret.init(hardwareMap);
        slides.init(hardwareMap);
        cameras.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
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
        chassis.run(gamepad1);
        claw.run(gamepad1);
        turret.run(gamepad1);
        slides.run(gamepad1);
        turret.lockOn(cameras.calculateMovement());
        if(gamepad1.b && slides.getCurrentPosition()>300){
            turret.setTurretPosition(0.5);
        }
        if(gamepad1.a) {
            FtcDashboard.getInstance().startCameraStream(cameras.getCamera(1), 0);
        }else {
            FtcDashboard.getInstance().startCameraStream(cameras.getCamera(2), 0);
        }
        if(chassis.getFrontDistance() < 12){
            if(!bliped){
                gamepad1.rumbleBlips(2);
                bliped = true;
            }
        }else{
            bliped = false;
        }
        telemetry.addData("Slide Position: ", slides.getCurrentPosition());
        telemetry.addData("Turret Position: ", turret.getTurretPosition());
        telemetry.addData("FL: ", chassis.getFrontLeftPosition());
        telemetry.addData("FR: ", chassis.getFrontRightPosition());
        telemetry.addData("BL: ", chassis.getBackLeftPosition());
        telemetry.addData("BR: ", chassis.getBackRightPosition());
        telemetry.addData("Angle: ", chassis.getAngle());
        telemetry.addData("Front: ", chassis.getFrontDistance());
        telemetry.addData("Back: ", chassis.getBackDistance());
        telemetry.addData("Yellow", cameras.getYellowLocation());
        telemetry.addData("cam1 -- position", cameras.yellowPos(1));
        telemetry.addData("cam2 -- position", cameras.yellowPos(2));
        telemetry.addData("cam 1 -- area", cameras.yellowArea(1));
        telemetry.addData("cam2 -- area", cameras.yellowArea(2));
        telemetry.addData("movement", cameras.calculateMovement());
        telemetry.addData("auto_adjust", turret.getAutoAdjust());
    }



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


}

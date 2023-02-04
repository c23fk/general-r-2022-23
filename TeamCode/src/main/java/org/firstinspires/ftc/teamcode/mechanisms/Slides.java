package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;

public class Slides implements Mechanism {
    private DcMotor slides = null;
    private int slidesTarget = 0;
    @Override
    public void init(HardwareMap hardwareMap){
        slides = hardwareMap.get(DcMotor.class, "slides");
        slides.setDirection(DcMotorSimple.Direction.FORWARD);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setTargetPosition(slidesTarget);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(Constants.SLIDE_POWER);
    }
    @Override
    public void run(Gamepad gamepad){
        if (gamepad.dpad_up) {
            slidesTarget = Constants.SLIDE_MAX;
        } else if (gamepad.dpad_right) {
            slidesTarget = Constants.MID_POSITION;
        } else if (gamepad.dpad_left) {
            slidesTarget = Constants.LOW_POSITION;
        } else if (gamepad.dpad_down) {
            slidesTarget = 0;
        }
        //manual adjustments to slide positions
        slidesTarget += -gamepad.right_stick_y * 50;
        slidesTarget = Range.clip(slidesTarget, -50, Constants.SLIDE_MAX);
        //move the slides
        slides.setTargetPosition(slidesTarget);
        slides.setPower(Constants.SLIDE_POWER);
        //reset the zero position of the slides
        if (gamepad.x) {
            slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public double getCurrentPosition() {
        return slidesTarget;
    }
}

package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Claw_2Drivers implements Mechanism {
    private Servo claw = null;
    private Servo wrist = null;
    private double clawPos;
    private double wristPos;

    @Override
    public void init(HardwareMap hardwareMap){
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        clawPos = Constants.CLAW_CLOSED;
        wristPos = 0.5;
        claw.setPosition(clawPos);
        wrist.setPosition(wristPos);
    }

    @Override
    public void run(Gamepad gamepad){
        if (gamepad.a) {
            clawPos = Constants.CLAW_CLOSED;
        } else if (gamepad.b) {
            clawPos = Constants.CLAW_OPEN;
        }
        if(gamepad.dpad_up){
            wristPos = Constants.WRIST_UP+0.1;
        }
        if(gamepad.dpad_down){
            wristPos = Constants.WRIST_DOWN;
        }

        if (gamepad.left_bumper) {
            wristPos = Constants.WRIST_UP + 0.1;
        } else if (gamepad.right_bumper) {
            wristPos = Constants.WRIST_DOWN;
        }
        clawPos -= 0.01 * gamepad.left_stick_y;
        claw.setPosition(clawPos);

        wristPos += 0.03 * (gamepad.right_trigger-gamepad.left_trigger);

        claw.setPosition(clawPos);
        wrist.setPosition(wristPos);
    }

    public double getClawPosition() {
        return clawPos;
    }

    public double getWristPosition() {
        return wristPos;
    }
}

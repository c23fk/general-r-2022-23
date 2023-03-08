package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Claw_1Driver implements Mechanism {
    private Servo claw = null;
    private Servo wrist = null;
    private double clawPos;
    private double wristPos;
    private boolean aClicked = false;

    @Override
    public void init(HardwareMap hardwareMap){
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        clawPos = Constants.CLAW_CLOSED;
        wristPos = Constants.WRIST_DOWN;
        claw.setPosition(clawPos);
        wrist.setPosition(wristPos);
    }

    @Override
    public void run(Gamepad gamepad){
        if (gamepad.a) {
            if(!aClicked) {
                clawPos = clawPos == Constants.CLAW_CLOSED?Constants.CLAW_OPEN:Constants.CLAW_CLOSED;
                aClicked = true;
            }
        } else {
            aClicked = false;
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

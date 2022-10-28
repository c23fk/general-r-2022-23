package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Claw implements Mechanism {
    private Servo claw = null;
    private Servo wrist = null;
    private double clawPos;
    private double wristPos;
    private boolean clawOpen;
    private boolean aPressed;

    @Override
    public void init(HardwareMap hardwareMap){
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        clawPos = Constants.CLAW_CLOSED;
        wristPos = 0.5;
        clawOpen = false;
        aPressed = false;
        claw.setPosition(clawPos);
        wrist.setPosition(wristPos);
    }

    @Override
    public void run(Gamepad gamepad){
        aPressed = false;
        if (gamepad.a & clawOpen & !aPressed) {
            clawPos = Constants.CLAW_CLOSED;
            clawOpen = false;
            aPressed = true;
        } else if (gamepad.a & !clawOpen & !aPressed) {
            clawPos = Constants.CLAW_OPEN;
            clawOpen = true;
            aPressed = true;
        }
        clawPos -= 0.01 * gamepad.left_stick_y;
        claw.setPosition(clawPos);

        if (gamepad.dpad_up) {
            wristPos += 0.01;
        } else if (gamepad.dpad_down) {
            wristPos -= 0.01;
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

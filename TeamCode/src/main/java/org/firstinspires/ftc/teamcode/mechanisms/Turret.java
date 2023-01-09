package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Turret implements Mechanism {
    private Servo turret = null;
    private double turretPos;

    @Override
    public void init(HardwareMap hardwareMap){
        turret = hardwareMap.get(Servo.class, "turret");
        turretPos = 0.5;
        turret.setPosition(turretPos);
    }

    @Override
    public void run(Gamepad gamepad){
        turretPos += Math.min(Math.max(0.001 * (gamepad.right_trigger-gamepad.left_trigger),-1),1);
//        TODO: add presets
        turret.setPosition(turretPos);
    }
    public double getTurretPosition() {
        return turretPos;
    }
}

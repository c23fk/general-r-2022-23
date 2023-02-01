package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Turret implements Mechanism {
    private Servo turret = null;
    private double turretPos;

    private boolean autoAdjust = false;

    @Override
    public void init(HardwareMap hardwareMap){
        turret = hardwareMap.get(Servo.class, "turret");
        turretPos = 0.5;
        turret.setPosition(turretPos);
    }

    @Override
    public void run(Gamepad gamepad){
        turretPos += Math.min(Math.max(0.001 * (gamepad.left_trigger-gamepad.right_trigger),-1),1);
//        TODO: add presets
        turret.setPosition(turretPos);
    }
    public double getTurretPosition() {
        return turretPos;
    }

    public void lockOn(double movement){
        if(autoAdjust) {
            turret.setPosition(turret.getPosition() + movement);
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    public void setAutoAdjust(boolean autoAdjust) {
        this.autoAdjust = autoAdjust;
    }

    public void setTurretPosition(double pos) {
        turretPos = pos;
        turret.setPosition(turretPos);
    }
}

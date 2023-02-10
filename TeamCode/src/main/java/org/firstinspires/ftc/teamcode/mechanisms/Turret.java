package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public  class Turret implements Mechanism {
    private Servo turret = null;
    private volatile double turretPos;

    private volatile boolean autoAdjust = false;

    @Override
    public void init(HardwareMap hardwareMap){
        turret = hardwareMap.get(Servo.class, "turret");
        turretPos = 0.5;
        turret.setPosition(turretPos);
    }

    @Override
    public void run(Gamepad gamepad){
        //turretPos += Math.min(Math.max(0.001 * (gamepad.left_trigger-gamepad.right_trigger),-1),1);
        if(gamepad.left_stick_y * gamepad.left_stick_y +gamepad.left_stick_x * gamepad.left_stick_x > 0.3){
            double turretAngle = Math.atan2(-gamepad.left_stick_y,gamepad.left_stick_x);
            turretPos = Math.min(0.057266 * turretAngle +0.41009,0.59);
        }
        turret.setPosition(turretPos);
    }
    public double getTurretPosition() {
        return turretPos;
    }

    public void lockOn(double movement){
        if(autoAdjust) {
            turret.setPosition(turret.getPosition() + movement);
            try {
                Thread.sleep(25);
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

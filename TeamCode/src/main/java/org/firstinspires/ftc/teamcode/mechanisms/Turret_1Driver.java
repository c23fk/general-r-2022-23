package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public  class Turret_1Driver implements Mechanism {
    private Servo turret = null;
    private boolean xClicked = false;
    private double turretPos;

    private volatile boolean autoAdjust = false;

    @Override
    public void init(HardwareMap hardwareMap){
        turret = hardwareMap.get(Servo.class, "turret");
        turretPos = 0.5;
        turret.setPosition(turretPos);
    }

    @Override
    public void run(Gamepad gamepad){
        if(gamepad.x){
            if(!xClicked) {
                setAutoAdjust(!autoAdjust);
                xClicked = true;
            }
        }else{
            xClicked = false;
        }
        if(gamepad.y){
            turretPos = 0.5;
            setAutoAdjust(false);
        }
        if(gamepad.dpad_down) {
            setAutoAdjust(false);
        }
        turret.setPosition(turretPos);
    }
    public double getTurretPosition() {
        return turretPos;
    }

    public void lockOn(double movement){
        if(autoAdjust) {
            turretPos = turret.getPosition() + movement;
//            try {
//                Thread.sleep(5);
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }
            System.out.println("locking");
        }
    }

    public void setAutoAdjust(boolean autoAdjust) {
        this.autoAdjust = autoAdjust;
    }
    public boolean getAutoAdjust(){
        return autoAdjust;
    }
    public void setTurretPosition(double pos) {
        turretPos = pos;
        turret.setPosition(turretPos);
    }
}

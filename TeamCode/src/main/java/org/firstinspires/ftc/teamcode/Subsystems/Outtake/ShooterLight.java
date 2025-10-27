package org.firstinspires.ftc.teamcode.Subsystems.Outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.lib.StateMachine.Action;

public class ShooterLight {
    Servo light;
    boolean blinkingOn = false;

    Action turnLedOn;

    Action turnLedOff;



    public ShooterLight(HardwareMap hardwareMap){
        light = hardwareMap.get(Servo.class, ShooterConstants.HMLight);
        turnLedOff = new Action(()->{
            blinkingOn = false;
            turnLedOn.Run(200);
        });

        turnLedOn = new Action(()->{
            blinkingOn = true;
            turnLedOff.Run(100);
        });
        turnLedOn.Run(0);
    }
    public enum MODES {
        OFF,
        BLINKING,
        ON
    }
    MODES mode = MODES.OFF;
    public void SetMode(MODES mode){
        this.mode = mode;
    }
    public void Loop(){
        switch (mode){
            case ON:
                light.setPosition(1);
                break;
            case OFF:
                light.setPosition(0);
                break;
            case BLINKING:
                if (blinkingOn){
                    light.setPosition(1);
                } else {
                    light.setPosition(0);
                }
                break;
        }
    }
}

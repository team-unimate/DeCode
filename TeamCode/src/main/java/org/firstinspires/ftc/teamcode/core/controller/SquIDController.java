package org.firstinspires.ftc.teamcode.core.controller;

public class SquIDController {
    private double kSQ;
    public double sp = 0;
    public SquIDController(double setkSQ){
        kSQ = setkSQ;
    }
    public void setSetpoint(double setPoint){
        sp =setPoint;
    }
    public double calculate(double currentPoint){
        return adjustedRoot(sp-currentPoint)*kSQ;
    }
    private double adjustedRoot(double num){
        if (num<0){
            return -Math.sqrt(num*-1);
        } else if (num>0){
            return Math.sqrt(num);
        }
        return 0;
    }
}

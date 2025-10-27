package org.firstinspires.ftc.teamcode.core.controller;

public class VelocityPIDf {
    private double kP, kI, kD, kV;
    private double integralSum = 0;
    private double lastError = 0;
    private double lastTime = 0;

    private double velocitySetpoint = 0;
    private double tolerance = 1e-2;
    private double batteryVoltage = 12.0; // tensão nominal padrão

    public VelocityPIDf(double kP, double kI, double kD, double kV) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kV = kV; // kV em power/ticksPerSecond (ex: 0.000353)
        this.lastTime = System.nanoTime() / 1e9;
    }

    public void setSetpoint(double setpoint) { this.velocitySetpoint = setpoint; }

    public double getSetpoint() { return velocitySetpoint; }

    public void setTolerance(double tolerance) { this.tolerance = tolerance; }

    public boolean atSetpoint(double currentVelocity) {
        return Math.abs(velocitySetpoint - currentVelocity) <= tolerance;
    }

    public void setBatteryVoltage(double voltage) { this.batteryVoltage = voltage; }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        lastTime = System.nanoTime() / 1e9;
    }

    public double calculate(double currentVelocity) {
        double currentTime = System.nanoTime() / 1e9;
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        double error = velocitySetpoint - currentVelocity;

        // Integral com anti-windup simples
        integralSum += error * dt;
        double derivative = (dt > 0) ? (error - lastError) / dt : 0;

        lastError = error;

        // Feedforward ajustado: (kV * velocitySetpoint) normalizado pela tensão atual
        double feedforward = (kV * velocitySetpoint) * (12.0 / batteryVoltage);

        double output = (kP * error) + (kI * integralSum) + (kD * derivative) + feedforward;

        // Limitando saída entre -1 e 1
        if (output > 1) output = 1;
        else if (output < -1) output = -1;

        return output;
    }
}

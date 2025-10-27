package org.firstinspires.ftc.teamcode.Subsystems.Outtake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.controller.VelocityPID;
import org.firstinspires.ftc.teamcode.core.lib.Subsystem;

public class ShooterSubsystem extends Subsystem {
    private DcMotorEx shooterMotor1;
    private Telemetry telemetry;
    ShooterLight light;
    private VelocityPID velocityPID;

    private double speedTolerance = 30; // RPM tolerância para "está na velocidade"
    private boolean isAtSpeed = false;
    private double map(double x, double inMin, double inMax, double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }
    public void AutoSetVelocity(double distance){
        double minDist = 58;
        double minDistVel = 890;
        double maxDist = 145;
        double maxDistVel = 1200;
        setTargetVelocityRPM(map(distance, minDist, maxDist, minDistVel, maxDistVel));


    }

    @Override
    public void Init(HardwareMap hardwareMap, Telemetry telemetry) {
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, ShooterConstants.HMShooterMotor);
        shooterMotor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER );

        shooterMotor1.setDirection(ShooterConstants.dir);
        light = new ShooterLight(hardwareMap);
        this.telemetry = telemetry;

        // Inicializa o controlador PID com valores que você deve ajustar/testar
        double kP = 0.07;  // exemplo de ganho proporcional
        double kI = 0.0000; // integral
        double kD = 0.0000; // derivativo
        double kF = 0.0;    // feedforward (opcional)

        velocityPID = new VelocityPID(kP, kI, kD, kF);
        velocityPID.setTolerance(speedTolerance);
        velocityPID.reset();
    }

    @Override
    public void Periodic() {
        // Velocidade atual do motor em RPM
        double currentVelocityRPM = shooterMotor1.getVelocity();

        // Calcula a saída do PID baseado na velocidade atual
        double power = velocityPID.calculate(currentVelocityRPM);

        // Limita power entre -1 e 1 para segurança
        power = Math.max(-1, Math.min(1, power));

        // Aplica potência ao motor
        shooterMotor1.setPower(power);

        // Verifica se está dentro da tolerância
        isAtSpeed = velocityPID.atSetpoint(currentVelocityRPM);

        if (isAtSpeed){
            light.SetMode(ShooterLight.MODES.ON);
        } else {
            light.SetMode(ShooterLight.MODES.BLINKING);
        }
        light.Loop();


    }
    public double currentSpeed(){
        return shooterMotor1.getVelocity();
    }
    public double targetSpeed(){
        return velocityPID.getSetpoint();
    }
    public void setTargetVelocityRPM(double RPM) {
        velocityPID.setSetpoint(RPM);
    }

    public boolean isAtSpeed() {
        return isAtSpeed;
    }

    public void resetPID() {
        velocityPID.reset();
    }
}

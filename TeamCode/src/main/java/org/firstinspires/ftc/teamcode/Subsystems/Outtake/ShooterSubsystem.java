package org.firstinspires.ftc.teamcode.Subsystems.Outtake;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.controller.VelocityPIDf;
import org.firstinspires.ftc.teamcode.core.lib.Subsystem;

public class ShooterSubsystem extends Subsystem {
    private DcMotorEx shooterMotor1;
    private Telemetry telemetry;
    ShooterLight light;
    public VelocityPIDf velocityPID;
    VoltageSensor voltageSensor;
    InterpLUT lut = new InterpLUT();
    double mindist = 55.4;
    double maxdist = 107;
    public ShooterSubsystem(){
        lut.add(mindist, 900.00);
        lut.add(92.23, 1025.00);
        lut.add(maxdist, 1100.00);
        lut.createLUT();

    }

    private double speedTolerance = 20; // RPM tolerância para "está na velocidade"
    private boolean isAtSpeed = false;
    private double map(double x, double inMin, double inMax, double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }
    public void AutoSetVelocity(double distance){
        if (distance<mindist){
            distance = mindist+1;
        } else if (distance>maxdist){
            distance = maxdist-1;
        }
        setTargetVelocity(lut.get(distance));
    }

    @Override
    public void Init(HardwareMap hardwareMap, Telemetry telemetry) {

        shooterMotor1 = hardwareMap.get(DcMotorEx.class, ShooterConstants.HMShooterMotor);
        shooterMotor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER );
        voltageSensor = hardwareMap.get(VoltageSensor.class,"Control Hub");

        shooterMotor1.setDirection(ShooterConstants.dir);
        light = new ShooterLight(hardwareMap);
        this.telemetry = telemetry;

        double kP = 0.07;  // exemplo de ganho proporcional
        double kI = 0.0000; // integral
        double kD = 0.0000; // derivativo
        double kV = 0.000353;    // feedforward (opcional)

        velocityPID = new VelocityPIDf(kP, kI, kD, kV);
        velocityPID.setTolerance(speedTolerance);
        velocityPID.reset();
    }

    @Override
    public void Periodic() {
        velocityPID.setBatteryVoltage(voltageSensor.getVoltage());
        double currentVelocityRPM = shooterMotor1.getVelocity();

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
    public void setTargetVelocity(double RPM) {
        velocityPID.setSetpoint(RPM);
    }

    public boolean isAtSpeed() {
        return isAtSpeed;
    }

    public void resetPID() {
        velocityPID.reset();
    }
}

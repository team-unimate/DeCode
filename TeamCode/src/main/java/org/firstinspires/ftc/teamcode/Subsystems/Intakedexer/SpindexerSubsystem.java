package org.firstinspires.ftc.teamcode.Subsystems.Intakedexer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.controller.PIDFController;
import org.firstinspires.ftc.teamcode.core.lib.Subsystem;

public class SpindexerSubsystem extends Subsystem {
    private DcMotor motorSpin;
    public double error;
    private int targetTicks = 0;
    private int currentTicks = 0;

    private final PIDFController pid = new PIDFController(0.0006, 0.0000000, 0.00002, 0); // ajuste fino

    private long lastTimeOutsideTolerance = 0; // último momento em que saiu da tolerância
    private static final int STABLE_TIME_MS = 150; // tempo necessário estável (ex: 300ms)
    private static final int TOLERANCE_TICKS = 250;

    public enum POSITIONS {
        INTAKE,
        OUTTAKE
    }
    private POSITIONS currentPosition = POSITIONS.INTAKE;

    public POSITIONS GetCurrentPosition() {
        return this.currentPosition;
    }

    public boolean IsSpinning() {
        int error = Math.abs(targetTicks - currentTicks);

        long now = System.currentTimeMillis();

        // Se ainda fora da tolerância, atualiza o tempo
        if (error > TOLERANCE_TICKS) {
            lastTimeOutsideTolerance = now;
        }

        // Retorna true se ainda não passou tempo suficiente desde que ficou dentro da tolerância
        return now - lastTimeOutsideTolerance < STABLE_TIME_MS;
    }

    @Override
    public void Init(HardwareMap hardwareMap, Telemetry telemetry) {
        motorSpin = hardwareMap.get(DcMotor.class, SpindexerConstants.HMEncoder);

        motorSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSpin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void Reset() {
        motorSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSpin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        currentPosition = POSITIONS.INTAKE;
        targetTicks = 0;
        currentTicks = 0;
        lastTimeOutsideTolerance = System.currentTimeMillis();
    }

    private void setTargetAngle(double angle) {
        setTargetPosition((int) (angle * (SpindexerConstants.TICKS_PER_REV / 360.0)));
    }

    public double getTargetPositionAsAngle() {
        return (double) targetTicks / (SpindexerConstants.TICKS_PER_REV / 360.0);
    }

    public void setTargetPosition(int position) {
        targetTicks = position;
        lastTimeOutsideTolerance = System.currentTimeMillis(); // reset ao definir novo alvo
    }

    public void Spin(POSITIONS position, int direction) { // direcao em 1 ou -1
        double currentAngle = getTargetPositionAsAngle();

        if (currentPosition == POSITIONS.INTAKE) {
            if (position == POSITIONS.INTAKE) {
                setTargetAngle(currentAngle + 120 * direction); // gira 120°
            } else {
                setTargetAngle(currentAngle + 60 * direction); // gira 60°
            }
        } else {
            if (position == POSITIONS.INTAKE) {
                setTargetAngle(currentAngle + 60 * direction);
            } else {
                setTargetAngle(currentAngle + 120 * direction);
            }
        }

        currentPosition = position;
    }

    @Override
    public void Periodic() {
        currentTicks = motorSpin.getCurrentPosition();
        error = targetTicks - currentTicks;
        pid.setSetpoint(targetTicks);
        double output = pid.calculate(currentTicks);

        // Limita a potência
        output = Math.max(-1, Math.min(1, output));

        motorSpin.setPower(output);
    }
}

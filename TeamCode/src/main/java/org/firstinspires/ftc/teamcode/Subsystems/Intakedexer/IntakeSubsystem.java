package org.firstinspires.ftc.teamcode.Subsystems.Intakedexer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.lib.Subsystem;

public class IntakeSubsystem extends Subsystem {
    private DcMotor intakeMotor;
    double power = 0.0;
    @Override
    public void Init(HardwareMap hardwareMap, Telemetry telemetry) {
        intakeMotor = hardwareMap.get(DcMotor.class, IntakeConstants.HMIntake);
    }

    public void TurnIntakeOn(){
        power = -0.65;
    }
    public void TurnIntakeOff(){
        power = 0;
    }
    public void TurnIntakeMeiaBomba(){
        power = 0.5;
    }
    @Override
    public void Periodic() {
       intakeMotor.setPower(power);

    }
}
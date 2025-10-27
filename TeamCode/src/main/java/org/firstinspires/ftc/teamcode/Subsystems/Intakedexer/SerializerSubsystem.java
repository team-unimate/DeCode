package org.firstinspires.ftc.teamcode.Subsystems.Intakedexer;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.controller.PIDFController;
import org.firstinspires.ftc.teamcode.core.lib.StateMachine.Action;
import org.firstinspires.ftc.teamcode.core.lib.Subsystem;

public class SerializerSubsystem extends Subsystem {
    Servo s1;
    Servo s2;
    private PIDFController PIDFController;
    private double targetVelocity = 0;
    private Telemetry telemetry;
    public boolean serializing = false;

    @Override
    public void Init(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;

        s1 = hardwareMap.get(Servo.class, SerializerConstants.HMSerializer1);
        s2 = hardwareMap.get(Servo.class, SerializerConstants.HMSerializer2);


        SetServosPosition(SerializerConstants.posDown);
    }
    public void Serialize(){
        SetServosPosition(SerializerConstants.posUp);
        serializing = true;
        Action downDone = new Action(()->{
            serializing = false;
        });

        Action goDown = new Action(()->{
            SetServosPosition(SerializerConstants.posDown);
            downDone.Run(SerializerConstants.delay);
        });
        goDown.Run(SerializerConstants.delay);
    }


    private void SetServosPosition(double position){
        s1.setPosition(position);
        s2.setPosition(position);
    }

    @Override
    public void Periodic() {


    }
}

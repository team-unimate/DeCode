package org.firstinspires.ftc.teamcode.Subsystems.Intakedexer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Intakedexer.Sensoring.ArtifactSensor;
import org.firstinspires.ftc.teamcode.core.lib.SubSystemGroup;
import org.firstinspires.ftc.teamcode.core.lib.Subsystem;

import java.util.ArrayList;
import java.util.List;

public class IntakeDexer extends SubSystemGroup {
    public List<ArtifactSensor.ARTIFACT_STATE> artifacts = new ArrayList<>();


    IntakeSubsystem intake;
    SpindexerSubsystem spindexer;
    ArtifactSensor sensorIntake;
    SerializerSubsystem serializer;
    boolean canShoot = false;
    public IntakeDexer(){
        intake = new IntakeSubsystem();
        spindexer = new SpindexerSubsystem();
        serializer = new SerializerSubsystem();
        sensorIntake = new ArtifactSensor("CI");
        Subsystem[] subsystems = {
                intake,
                spindexer,
                sensorIntake,
                serializer
        };
        scheduleSubsystems(subsystems);
    }
    public void Reset(){
        IntakeThreeMode();
        spindexer.Reset();
    }
    public enum STATES{
        IDLE,
        INTAKE_THREE,
        OUTTAKE_THREE,
        ALTERNATE
    }
    public boolean CanSerialize(){
        return !spindexer.IsSpinning() && spindexer.GetCurrentPosition()== SpindexerSubsystem.POSITIONS.OUTTAKE;
    }
    public boolean CanIntake(){
        return !spindexer.IsSpinning() && spindexer.GetCurrentPosition()== SpindexerSubsystem.POSITIONS.INTAKE;
    }
    public void Serialize(){
        serializer.Serialize();
    }

    public STATES currentState = STATES.IDLE;
    int InOutCount = 1;
    private void SetState(STATES state){
        this.currentState = state;
    }
    enum ALTERNATE_STATES{
        INTAKE,
        OUTTAKE
    }
    ALTERNATE_STATES alternateState = ALTERNATE_STATES.INTAKE;
    public void AlternateMode(){
        currentState = STATES.ALTERNATE;
        alternateState = ALTERNATE_STATES.INTAKE;
        spindexer.Spin(SpindexerSubsystem.POSITIONS.INTAKE, 1);
    }
    public void IntakeThreeMode(){
        artifacts.clear();
        if (currentState==STATES.IDLE) {
            InOutCount = 1;
            SetState(STATES.INTAKE_THREE);
            spindexer.Spin(SpindexerSubsystem.POSITIONS.INTAKE, 1);
        }
    }
    int SerializedAt = 0;
    public void SerializeThreeMode(){
        balCount = 3;
        artifacts.clear();
         if (currentState!=STATES.OUTTAKE_THREE) {
             InOutCount = 1;
             SetState(STATES.OUTTAKE_THREE);
             spindexer.Spin(SpindexerSubsystem.POSITIONS.OUTTAKE, 1);
             SerializedAt = 0;
         }
    }
    int balCount = 0;
    public void setCanShoot(boolean set){
        canShoot = set;
    }

    boolean serialized = false;

    @Override
    public void GroupPeriodic(Telemetry telemetry) {
        switch (currentState) {
            case INTAKE_THREE:
                intake.TurnIntakeOn();
                if (InOutCount <= 3) {
                    if (!spindexer.IsSpinning()) {
                        if (sensorIntake.GetArtifact() != ArtifactSensor.ARTIFACT_STATE.NONE) {
                            artifacts.add(sensorIntake.GetArtifact());
                            balCount++;
                            spindexer.Spin(SpindexerSubsystem.POSITIONS.INTAKE, 1);
                            InOutCount++;
                        }
                    }
                } else {
                    SetState(STATES.IDLE);
                }
                break;
            case OUTTAKE_THREE:
                if (InOutCount <= 3) {

                    if (SerializedAt < InOutCount) {
                        if (!spindexer.IsSpinning() && !serializer.serializing && canShoot) {
                            serializer.Serialize();
                            SerializedAt++;
                        }
                    }
                    else if (!serializer.serializing && !spindexer.IsSpinning()) {
                        spindexer.Spin(SpindexerSubsystem.POSITIONS.OUTTAKE, 1);
                        InOutCount++;
                    }
                } else {
                    balCount=0;
                    SetState(STATES.IDLE);
                }
                break;
            case ALTERNATE:
                if (alternateState==ALTERNATE_STATES.INTAKE && !spindexer.IsSpinning() && sensorIntake.GetArtifact()!= ArtifactSensor.ARTIFACT_STATE.NONE){
                    serialized = false;
                    spindexer.Spin(SpindexerSubsystem.POSITIONS.OUTTAKE, 1);
                    alternateState=ALTERNATE_STATES.OUTTAKE;
                } else if (alternateState==ALTERNATE_STATES.OUTTAKE && !spindexer.IsSpinning()) {
                    if (!serialized && canShoot) {
                        serializer.Serialize();
                        serialized=true;
                    }
                    if (serialized && !serializer.serializing){
                        alternateState = ALTERNATE_STATES.INTAKE;
                        spindexer.Spin(SpindexerSubsystem.POSITIONS.INTAKE, 1);
                    }
                }
                break;
            case IDLE:
                break;

        }
        intake.TurnIntakeOn();
            /*if (spindexer.IsSpinning()){
                intake.TurnIntakeOn();
            } else if (this.currentState!=STATES.INTAKE_THREE){
                intake.TurnIntakeOff();
            }
            if (this.currentState==STATES.ALTERNATE && !serializer.serializing){
                intake.TurnIntakeOn();
            } else {
                intake.TurnIntakeOff();
            }
            if (currentState==STATES.INTAKE_THREE){
                intake.TurnIntakeOn();
            }*/
        telemetry.addData("Spindexer ok: ", !spindexer.IsSpinning());

    }
    public void ForceIndexNextBall(){
        serialized = false;
        spindexer.Spin(SpindexerSubsystem.POSITIONS.OUTTAKE, 1);
        alternateState=ALTERNATE_STATES.OUTTAKE;
    }
    public int bals(){
        return balCount;
    }
    public double currentServo(){
        return serializer.s1.getPosition();
    }
}

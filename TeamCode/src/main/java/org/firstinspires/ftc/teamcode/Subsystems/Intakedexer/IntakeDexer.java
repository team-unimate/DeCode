package org.firstinspires.ftc.teamcode.Subsystems.Intakedexer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Intakedexer.Sensoring.ArtifactSensor;
import org.firstinspires.ftc.teamcode.Patterns;
import org.firstinspires.ftc.teamcode.core.lib.SubSystemGroup;
import org.firstinspires.ftc.teamcode.core.lib.Subsystem;

import java.util.ArrayList;
import java.util.List;

public class IntakeDexer extends SubSystemGroup {

    IntakeSubsystem intake;
    SpindexerSubsystem spindexer;
    ArtifactSensor sensorIntake;
    ArtifactSensor sensorOuttake;
    SerializerSubsystem serializer;
    boolean canShoot = false;



    public IntakeDexer() {
        intake = new IntakeSubsystem();
        spindexer = new SpindexerSubsystem();
        serializer = new SerializerSubsystem();
        sensorIntake = new ArtifactSensor("CI");
        sensorOuttake = new ArtifactSensor("CO");
        Subsystem[] subsystems = {
                intake,
                spindexer,
                sensorIntake,
                sensorOuttake,
                serializer
        };
        scheduleSubsystems(subsystems);
    }

    public void Reset() {
        balCount = 3;

        spindexer.Reset();
    }

    public enum STATES {
        IDLE,
        INTAKE_THREE,
        OUTTAKE_THREE,
        ALTERNATE
    }

    public boolean CanSerialize() {
        return !spindexer.IsSpinning() && spindexer.GetCurrentPosition() == SpindexerSubsystem.POSITIONS.OUTTAKE;
    }

    public boolean CanIntake() {
        return !spindexer.IsSpinning() && spindexer.GetCurrentPosition() == SpindexerSubsystem.POSITIONS.INTAKE;
    }

    public void Serialize() {
        serializer.Serialize();
    }

    public STATES currentState = STATES.IDLE;
    int InOutCount = 1;

    private void SetState(STATES state) {
        this.currentState = state;
    }

    enum ALTERNATE_STATES {
        INTAKE,
        OUTTAKE
    }

    ALTERNATE_STATES alternateState = ALTERNATE_STATES.INTAKE;

    public void AlternateMode() {
        currentState = STATES.ALTERNATE;
        alternateState = ALTERNATE_STATES.INTAKE;
        SpinAndReorder(SpindexerSubsystem.POSITIONS.INTAKE);
    }

    public void IntakeThreeMode() {
        if (currentState == STATES.IDLE) {
            InOutCount = 1;
            SetState(STATES.INTAKE_THREE);
            if (spindexer.GetCurrentPosition()!= SpindexerSubsystem.POSITIONS.INTAKE) {
                spindexer.Spin(SpindexerSubsystem.POSITIONS.INTAKE, 1);
            }
        }
    }

    int SerializedAt = 0;

    public void SerializeThreeMode(SHOOTING_MODES mode) {
        balCount = 3;
        if (currentState != STATES.OUTTAKE_THREE) {
            currentShootingMode = mode;
            patternIndex = 1;
            InOutCount = 1;
            if (spindexer.GetCurrentPosition()!= SpindexerSubsystem.POSITIONS.OUTTAKE) {
                spindexer.Spin(SpindexerSubsystem.POSITIONS.OUTTAKE, 1);
            }
            SetState(STATES.OUTTAKE_THREE);

            SerializedAt = 0;
        }
    }

    int balCount = 0;

    public void setCanShoot(boolean set) {
        canShoot = set;
    }

    boolean serialized = false;

    public Patterns currentPattern =Patterns.PGP;
    public void SetPattern(Patterns pattern){
        currentPattern = pattern;
    }
    public int patternIndex = 1;
    boolean ignorePattern = false;

    public ArtifactSensor.ARTIFACT_STATE slot1 = ArtifactSensor.ARTIFACT_STATE.NONE;
    public ArtifactSensor.ARTIFACT_STATE slot2 = ArtifactSensor.ARTIFACT_STATE.NONE;
    public ArtifactSensor.ARTIFACT_STATE slot3 = ArtifactSensor.ARTIFACT_STATE.NONE;
    private ArtifactSensor.ARTIFACT_STATE GetPatternColor(Patterns pattern, int index) {
        switch (pattern) {
            case PGP:
                if (index == 1) return ArtifactSensor.ARTIFACT_STATE.PURPLE;
                if (index == 2) return ArtifactSensor.ARTIFACT_STATE.GREEN;
                if (index == 3) return ArtifactSensor.ARTIFACT_STATE.PURPLE;
                break;
            case PPG:
                if (index == 1) return ArtifactSensor.ARTIFACT_STATE.PURPLE;
                if (index == 2) return ArtifactSensor.ARTIFACT_STATE.PURPLE;
                if (index == 3) return ArtifactSensor.ARTIFACT_STATE.GREEN;
                break;
            case GPP:
                if (index == 1) return ArtifactSensor.ARTIFACT_STATE.GREEN;
                if (index == 2) return ArtifactSensor.ARTIFACT_STATE.PURPLE;
                if (index == 3) return ArtifactSensor.ARTIFACT_STATE.PURPLE;
                break;
            default:
                break;
        }
        return ArtifactSensor.ARTIFACT_STATE.NONE;
    }

    public void SetInitialArtifacts(ArtifactSensor.ARTIFACT_STATE first, ArtifactSensor.ARTIFACT_STATE second, ArtifactSensor.ARTIFACT_STATE third) {

        balCount = 3;
    }

    private void SpinAndReorder(SpindexerSubsystem.POSITIONS position) {
        spindexer.Spin(position, 1);
        ArtifactSensor.ARTIFACT_STATE s1 = slot1;
        ArtifactSensor.ARTIFACT_STATE s2 = slot2;
        ArtifactSensor.ARTIFACT_STATE s3 = slot3;
        slot1 = s3;
        slot2 = s1;
        slot3 = s2;
    }
    public void IndexBallFromSlot(int slot){


            switch (slot){
                case 1:
                    SpinAndReorder(SpindexerSubsystem.POSITIONS.OUTTAKE);
                    break;
                case 2:
                    SpinAndReorder(SpindexerSubsystem.POSITIONS.OUTTAKE);
                    SpinAndReorder(SpindexerSubsystem.POSITIONS.OUTTAKE);
                    break;
                case 3:
                    SpinAndReorder(SpindexerSubsystem.POSITIONS.OUTTAKE);
                    SpinAndReorder(SpindexerSubsystem.POSITIONS.OUTTAKE);
                    SpinAndReorder(SpindexerSubsystem.POSITIONS.OUTTAKE);
                    break;
            }
    }

    public enum SHOOTING_MODES{
        PATTERN,
        RANDOM
    }
    SHOOTING_MODES currentShootingMode = SHOOTING_MODES.RANDOM;

    public int GetSlotFromBall(ArtifactSensor.ARTIFACT_STATE ball){
        if (slot1==ball){
            return 1;
        } else if (slot2==ball){
            return 2;
        }else if (slot3==ball){
            return 3;
        }
        return -1;
    }
    public void SetSlotState(int slot, ArtifactSensor.ARTIFACT_STATE ball){
        if (slot==1){
            slot1 = ball;
        } else if (slot==2){
            slot2 = ball;
        }else if (slot==3){
            slot3 = ball;
        }
    }
    int slotIndex =1;
    @Override
    public void GroupPeriodic(Telemetry telemetry) {
        switch (currentState) {
            case INTAKE_THREE:

                    intake.TurnIntakeOn();
                    if (InOutCount <= 3) {
                        ArtifactSensor.ARTIFACT_STATE newBall = sensorIntake.GetArtifact();
                        if (!spindexer.IsSpinning() && newBall!= ArtifactSensor.ARTIFACT_STATE.NONE) {
                            balCount++;
                            slot1 = newBall;
                            SpinAndReorder(SpindexerSubsystem.POSITIONS.INTAKE);
                            InOutCount++;
                        }
                    } else {
                        SetState(STATES.IDLE);
                    }

                break;

            case OUTTAKE_THREE:
                if (currentShootingMode==SHOOTING_MODES.RANDOM) {
                    if (InOutCount <= 3) {
                        if (SerializedAt < InOutCount) {
                            if (!spindexer.IsSpinning() && !serializer.serializing && canShoot) {
                                serializer.Serialize();

                                SerializedAt++;
                            }
                        } else if (!serializer.serializing && !spindexer.IsSpinning()) {
                            SpinAndReorder(SpindexerSubsystem.POSITIONS.OUTTAKE);

                            InOutCount++;
                        }
                    } else {
                        balCount = 0;
                        SetState(STATES.IDLE);
                    }
                } else {
                    if (InOutCount <= 3) {
                        if (SerializedAt < InOutCount) {
                            if (!spindexer.IsSpinning() && !serializer.serializing && canShoot) {

                                serializer.Serialize();
                                SetSlotState(2  , ArtifactSensor.ARTIFACT_STATE.NONE);
                                SerializedAt++;
                            }
                        } else if (!serializer.serializing && !spindexer.IsSpinning()) {
                            ArtifactSensor.ARTIFACT_STATE nextBallColor = GetPatternColor(currentPattern, patternIndex);
                            slotIndex = GetSlotFromBall(nextBallColor);

                            if (slotIndex == -1) {
                                currentShootingMode=SHOOTING_MODES.RANDOM;
                                return;
                            } else {

                                IndexBallFromSlot(slotIndex);

                            }

                            patternIndex++;
                            if (patternIndex > 3) patternIndex = 1;

                            InOutCount++;
                        }
                    } else {
                        balCount = 0;
                        SetState(STATES.IDLE);
                    }
                }
                break;

            case ALTERNATE:
                if (alternateState == ALTERNATE_STATES.INTAKE &&
                        !spindexer.IsSpinning() &&
                        sensorIntake.GetArtifact() != ArtifactSensor.ARTIFACT_STATE.NONE) {
                    serialized = false;
                    SpinAndReorder(SpindexerSubsystem.POSITIONS.OUTTAKE);
                    
                    alternateState = ALTERNATE_STATES.OUTTAKE;
                } else if (alternateState == ALTERNATE_STATES.OUTTAKE && !spindexer.IsSpinning()) {
                    if (!serialized && canShoot) {
                        serializer.Serialize();

                        serialized = true;
                    }
                    if (serialized && !serializer.serializing) {
                        alternateState = ALTERNATE_STATES.INTAKE;
                        SpinAndReorder(SpindexerSubsystem.POSITIONS.INTAKE);
                        
                    }
                }
                break;
            case IDLE:
                break;
        }
        telemetry.addData("Ballcount: ", bals());
        telemetry.addData("Ball1: ", slot1);
        telemetry.addData("Ball2: ", slot2);
        telemetry.addData("Ball3: ", slot3);
        telemetry.addData("GOAL BALL: ", GetPatternColor(currentPattern, patternIndex));
        telemetry.addData("State: ", currentState);
        telemetry.addData("Shooting mode: ", currentShootingMode);
        intake.TurnIntakeOn();
    }

    public void ForceIndexNextBall() {
        serialized = false;

        SpinAndReorder(SpindexerSubsystem.POSITIONS.OUTTAKE);
        
        alternateState = ALTERNATE_STATES.OUTTAKE;
    }

    public int bals() {
        return balCount;
    }

    public double currentServo() {
        return serializer.s1.getPosition();
    }

}

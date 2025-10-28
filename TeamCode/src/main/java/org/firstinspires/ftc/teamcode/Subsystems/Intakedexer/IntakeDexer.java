package org.firstinspires.ftc.teamcode.Subsystems.Intakedexer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Intakedexer.Sensoring.ArtifactSensor;
import org.firstinspires.ftc.teamcode.Patterns;
import org.firstinspires.ftc.teamcode.core.lib.SubSystemGroup;
import org.firstinspires.ftc.teamcode.core.lib.Subsystem;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

public class IntakeDexer extends SubSystemGroup {
    public LinkedList<ArtifactSensor.ARTIFACT_STATE> artifacts = new LinkedList<>();

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
        IntakeThreeMode();
        spindexer.Reset();
    }

    public enum STATES {
        IDLE,
        INTAKE_THREE,
        OUTTAKE_THREE,
        ALTERNATE,
        SHOOT_PATTERN
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
        artifacts.clear();
        if (currentState == STATES.IDLE) {
            InOutCount = 1;
            SetState(STATES.INTAKE_THREE);
            SpinAndReorder(SpindexerSubsystem.POSITIONS.INTAKE);
        }
    }

    int SerializedAt = 0;

    public void SerializeThreeMode() {
        balCount = 3;
        artifacts.clear();
        if (currentState != STATES.OUTTAKE_THREE) {
            InOutCount = 1;
            SetState(STATES.OUTTAKE_THREE);
            SpinAndReorder(SpindexerSubsystem.POSITIONS.OUTTAKE);
            SerializedAt = 0;
        }
    }

    int balCount = 0;

    public void setCanShoot(boolean set) {
        canShoot = set;
    }

    boolean serialized = false;

    Patterns currentPattern = null;
    int patternIndex = 0;
    boolean ignorePattern = false;
    public void ShootPattern(Patterns pattern) {
        if (currentState!=STATES.SHOOT_PATTERN) {
            currentPattern = pattern;
            patternIndex = 0;
            balCount = 3;
            InOutCount = 1;
            patternAttempts = 0;
            SetState(STATES.SHOOT_PATTERN);
            ignorePattern = false;

            spindexer.Spin(SpindexerSubsystem.POSITIONS.OUTTAKE, 1);
        }
        }


    private ArtifactSensor.ARTIFACT_STATE GetPatternColor(Patterns pattern, int index) {
        switch (pattern) {
            case PGP:
                if (index == 0) return ArtifactSensor.ARTIFACT_STATE.PURPLE;
                if (index == 1) return ArtifactSensor.ARTIFACT_STATE.GREEN;
                if (index == 2) return ArtifactSensor.ARTIFACT_STATE.PURPLE;
                break;
            case PPG:
                if (index == 0) return ArtifactSensor.ARTIFACT_STATE.PURPLE;
                if (index == 1) return ArtifactSensor.ARTIFACT_STATE.PURPLE;
                if (index == 2) return ArtifactSensor.ARTIFACT_STATE.GREEN;
                break;
            case GPP:
                if (index == 0) return ArtifactSensor.ARTIFACT_STATE.GREEN;
                if (index == 1) return ArtifactSensor.ARTIFACT_STATE.PURPLE;
                if (index == 2) return ArtifactSensor.ARTIFACT_STATE.PURPLE;
                break;
            default:
                break;
        }
        return ArtifactSensor.ARTIFACT_STATE.NONE;
    }

    public void SetInitialArtifacts(ArtifactSensor.ARTIFACT_STATE first, ArtifactSensor.ARTIFACT_STATE second, ArtifactSensor.ARTIFACT_STATE third) {
        artifacts.clear();
        artifacts.add(first);
        artifacts.add(second);
        artifacts.add(third);
        balCount = 3;
    }

    private void SpinAndReorder(SpindexerSubsystem.POSITIONS position) {
        spindexer.Spin(position, 1);
        if (!artifacts.isEmpty()) {
            ArtifactSensor.ARTIFACT_STATE top = artifacts.removeLast();
            artifacts.addFirst(top);
        }
    }



    private int patternAttempts = 0;      // quantas vezes girou tentando achar a bola correta
    private boolean waitingForBall = false; // indica se estamos aguardando sensor
    private boolean ballFound = false;      // indica se a bola correta foi encontrada
    private long waitStartTime = 0;


    @Override
    public void GroupPeriodic(Telemetry telemetry) {
        switch (currentState) {
            case INTAKE_THREE:
                intake.TurnIntakeOn();
                if (InOutCount <= 3) {
                    if (!spindexer.IsSpinning()) {
                        ArtifactSensor.ARTIFACT_STATE newBall = sensorIntake.GetArtifact();
                        if (newBall != ArtifactSensor.ARTIFACT_STATE.NONE) {
                            artifacts.add(newBall);
                            balCount++;
                            SpinAndReorder(SpindexerSubsystem.POSITIONS.INTAKE);
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
                    } else if (!serializer.serializing && !spindexer.IsSpinning()) {
                        SpinAndReorder(SpindexerSubsystem.POSITIONS.OUTTAKE);
                        InOutCount++;
                    }
                } else {
                    balCount = 0;
                    SetState(STATES.IDLE);
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

            case SHOOT_PATTERN:
                if (InOutCount <= 3) {
                    if (SerializedAt < InOutCount) {
                        if (!spindexer.IsSpinning() && !serializer.serializing && canShoot) {

                            if (ignorePattern) {
                                serializer.Serialize();
                                SerializedAt++;
                            } else {
                                if (!waitingForBall) {
                                    waitStartTime = System.currentTimeMillis();
                                    waitingForBall = true;
                                } else if (System.currentTimeMillis() - waitStartTime >= 50) {
                                    ArtifactSensor.ARTIFACT_STATE detected = sensorOuttake.GetArtifact();
                                    if (detected == GetPatternColor(currentPattern, patternIndex)) {
                                        patternIndex++;
                                        serializer.Serialize();
                                        SerializedAt++;
                                    } else {
                                        // bola errada, pular para a próxima
                                        patternAttempts++;
                                        SpinAndReorder(SpindexerSubsystem.POSITIONS.OUTTAKE); // gira para a próxima bola
                                    }
                                    waitingForBall = false; // reseta espera para a próxima bola
                                }

                            }

                        }
                    } else if (!serializer.serializing && !spindexer.IsSpinning()) {
                        SpinAndReorder(SpindexerSubsystem.POSITIONS.OUTTAKE);
                        InOutCount++;
                    }
                } else {
                    balCount = 0;
                    SetState(STATES.IDLE);
                }
                break;


            case IDLE:
                break;
        }
        telemetry.addData("GOALBALL: ", GetPatternColor(Patterns.PGP, patternIndex));
        telemetry.addData("OUTTAKE SENSOR: ", sensorOuttake.GetArtifact());
        intake.TurnIntakeOn();
        telemetry.addData("BALLS: ", artifacts);
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

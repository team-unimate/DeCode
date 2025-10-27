package org.firstinspires.ftc.teamcode.Subsystems.Intakedexer.Sensoring;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.lib.Subsystem;

import java.util.logging.Handler;

public class ArtifactSensor extends Subsystem {
    RevColorSensorV3 colorSensor;
    ColorInterval greenArtifact = new ColorInterval(0.01,0.02,0.03,1,0.03,0.07);
    ColorInterval purpleArtifact = new ColorInterval(0.02,1,0.02,0.04,0.04,1);

    double maxDist = 1.4;
    public enum ARTIFACT_STATE {
        PURPLE,
        GREEN,
        UNKNOWN,
        NONE
    };
    String HMartifact;
    public ArtifactSensor(String HMartifact){
        this.HMartifact = HMartifact;
    }

    @Override
    public void Init(HardwareMap hardwareMap, Telemetry telemetry){
        colorSensor = hardwareMap.get(RevColorSensorV3.class, HMartifact);
    }
    @Override
    public void Periodic(){

    }
    public ARTIFACT_STATE GetArtifact(){
        ARTIFACT_STATE state = ARTIFACT_STATE.NONE;
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        boolean isPurple = purpleArtifact.IsCoincident(colors);
        boolean isGreen = greenArtifact.IsCoincident(colors);
        double r =  (double)(colors.red );
        double g = (double)(colors.green);
        double b =(double) (colors.blue);
        double distance = colorSensor.getDistance(DistanceUnit.CM);



        if (distance<maxDist){
            if (g>b){
                state = ARTIFACT_STATE.GREEN;
            } else {
                state = ARTIFACT_STATE.PURPLE;
            }
        } else {
            state = ARTIFACT_STATE.NONE;
        }
        return state;

    }
    public double dist(){
        return colorSensor.getDistance(DistanceUnit.CM);
    }

}

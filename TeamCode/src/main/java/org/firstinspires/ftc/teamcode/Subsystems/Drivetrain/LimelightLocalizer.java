package org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.core.lib.Subsystem;

public class LimelightLocalizer extends Subsystem {
    Limelight3A limelight;
    @Override
    public void Init(HardwareMap hardwareMap, Telemetry telemetry) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
    }
    Pose LastPose = new Pose(0,0,0);
    public void SetPose(Pose pose){
        this.LastPose = pose;
    }
    public Pose3D GetPoseByVision() {
        // Envia a orientação da IMU para a Limelight para obter a melhor pose (MegaTag2)
        limelight.updateRobotOrientation(Math.toDegrees(LastPose.getHeading()));

        LLResult result = limelight.getLatestResult();
        // Verificação de Confiança
        if (result == null && result.isValid()
                | result.getStaleness() > 200) {
        }

        Pose3D visionPose3D = result.getBotpose_MT2();
        return visionPose3D;
    }


    @Override
    public void Periodic() {

    }
}

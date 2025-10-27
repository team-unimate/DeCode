package org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.lib.Subsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class DTFollower extends Subsystem {
    Follower follower;
    @Override
    public void Init(HardwareMap hardwareMap, Telemetry telemetry) {
        follower = Constants.createFollower(hardwareMap);
    }
    public Follower getFollower(){
        return follower;
    }

    @Override
    public void Periodic() {
        follower.update();
    }
}

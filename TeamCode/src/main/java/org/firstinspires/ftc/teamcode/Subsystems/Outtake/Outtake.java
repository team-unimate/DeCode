package org.firstinspires.ftc.teamcode.Subsystems.Outtake;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.lib.SubSystemGroup;
import org.firstinspires.ftc.teamcode.core.lib.Subsystem;

public class Outtake extends SubSystemGroup {
    public TurretSubsystem turret;
    ShooterSubsystem shooter;
    public Outtake(){
        turret = new TurretSubsystem();
        shooter = new ShooterSubsystem();
        Subsystem[] subsystems = {
                turret, shooter
        };
        scheduleSubsystems(subsystems);
    }
    public boolean canShoot(){
        return shooter.isAtSpeed()&&turret.stabilized();
    }
    TurretSubsystem.GOALS goal = TurretSubsystem.GOALS.BLUE_GOAL;
    public void SetGoal(TurretSubsystem.GOALS goal){
        this.goal = goal;
    }
    Pose poseBot = new Pose(0,0,0);
    public void SetPose(Pose pose){
        poseBot = pose;
    }

    public enum STATES {
        FLYWHEEL_ON,
        FLYWHEEL_OFF
    }
    public void Reset(){

        turret.Reset();
    }
    STATES state = STATES.FLYWHEEL_OFF;
    public void TurnFlywheelOn(){
        state = STATES.FLYWHEEL_ON;
    }
    public void TurnFlywheelOff(){
        state = STATES.FLYWHEEL_OFF;
    }
    @Override
    public void GroupPeriodic(Telemetry telemetry) {
        turret.AimTo(goal, poseBot);
        switch (state){
            case FLYWHEEL_ON:
                shooter.AutoSetVelocity(turret.distanceToGoal);
                break;
            case FLYWHEEL_OFF:
                shooter.setTargetVelocityRPM(400);

                break;
        }
        telemetry.addData("Turret ok: ", turret.stabilized());
        telemetry.addData("Shooter ok: ", shooter.isAtSpeed());
        telemetry.addData("Shooter target speed: ", shooter.targetSpeed());
        telemetry.addData("Shooter actual speed: ", shooter.currentSpeed());
    }

}

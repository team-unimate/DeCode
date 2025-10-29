package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intakedexer.IntakeDexer;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake.TurretSubsystem;
import org.firstinspires.ftc.teamcode.core.lib.Robot;
import org.firstinspires.ftc.teamcode.core.lib.SubSystemGroup;

public class MainRobot extends Robot {
    /*
    Instance your subsystem groups here, example:
    public static Drivetrain drive = new Drivetrain();
    */
    public static final IntakeDexer intakeDexer = new IntakeDexer();
    public static final Outtake outtake = new Outtake();
    public static final Drivetrain drivetrain = new Drivetrain();
    public static SubSystemGroup[] subSystemGroups = {
            intakeDexer,
            outtake,
            drivetrain
            /*
            Place your SubsystemGroups here, example:
            drive,
            */
    };

    public MainRobot() {
        super(MainRobot.subSystemGroups);
    }

    public void SetPattern(Patterns pattern){
        intakeDexer.SetPattern(pattern);
    }


    public void ForceShoot(){
        intakeDexer.SerializeThreeMode(shootingMode);
    }
    public void SetGoal(TurretSubsystem.GOALS goal){
        outtake.SetGoal(goal);
    }

    public void setShootingMode(IntakeDexer.SHOOTING_MODES shootingMode){
        this.shootingMode = shootingMode;

    }

    IntakeDexer.SHOOTING_MODES shootingMode = IntakeDexer.SHOOTING_MODES.RANDOM;
    private boolean AutoShooting = false;
    public void SetAutoShooting(boolean AutoShooting){
        this.AutoShooting = AutoShooting;
    }
    @Override
    public void RobotPeriodic(Telemetry telemetry) {
        intakeDexer.setCanShoot(outtake.canShoot()&& drivetrain.isAtShootingZone());
        outtake.SetPose(drivetrain.getFollower().getPose());


        outtake.TurnFlywheelOn();

        if (outtake.getPattern()!=null){
            intakeDexer.SetPattern(outtake.getPattern());
        }


        if (intakeDexer.currentState!= IntakeDexer.STATES.ALTERNATE) {
        //    intakeDexer.AlternateMode();
        }

        if (drivetrain.isAtShootingZone()){
            if (intakeDexer.currentState== IntakeDexer.STATES.OUTTAKE_THREE){
                drivetrain.SetTeleOpState(Drivetrain.TELEOP_STATES.SHOOTING);
            } else {
                drivetrain.SetTeleOpState(Drivetrain.TELEOP_STATES.NORMAL);
            }


        } else {
            drivetrain.SetTeleOpState(Drivetrain.TELEOP_STATES.NORMAL);
        }
        if (intakeDexer.bals()>=3){
            intakeDexer.SerializeThreeMode(shootingMode);

        } else {
            intakeDexer.IntakeThreeMode();
        }
        telemetry.addData("SHooting mode do mainrobot: ", shootingMode);
        telemetry.addData("MOTIF: ", outtake.getPattern());
        telemetry.addData("PATTERNINDEX: ", intakeDexer.patternIndex);
        telemetry.addData("STATE DO SPINDEXR: ", intakeDexer.currentState);
        telemetry.update();

    }
    public boolean isShootingDone() {
        return intakeDexer.currentState != IntakeDexer.STATES.OUTTAKE_THREE;
    }


}

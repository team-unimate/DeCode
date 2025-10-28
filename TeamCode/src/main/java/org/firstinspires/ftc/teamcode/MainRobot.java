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
    Patterns pattern = Patterns.PGP;
    public void SetPattern(Patterns pattern){
        this.pattern = pattern;
    }

    public void ForceShootOnPattern(){
        intakeDexer.ShootPattern(pattern);
    }
    public void ForceShoot(){
        intakeDexer.SerializeThreeMode();
    }
    public void SetGoal(TurretSubsystem.GOALS goal){
        outtake.SetGoal(goal);
    }
    public enum SHOOTING_MODES{
        PATTERN,
        RANDOM
    }
    public void setShootingMode(SHOOTING_MODES shootingMode){
        this.shootingMode = shootingMode;
    }
    SHOOTING_MODES shootingMode = SHOOTING_MODES.RANDOM;
    @Override
    public void RobotPeriodic(Telemetry telemetry) {
        intakeDexer.setCanShoot(outtake.canShoot()&& drivetrain.isAtShootingZone());
        outtake.SetPose(drivetrain.getFollower().getPose());

        outtake.TurnFlywheelOn();
        if (intakeDexer.currentState!= IntakeDexer.STATES.ALTERNATE) {
        //    intakeDexer.AlternateMode();
        }

        if (drivetrain.isAtShootingZone()){
            if (intakeDexer.currentState== IntakeDexer.STATES.OUTTAKE_THREE){
                drivetrain.SetTeleOpState(Drivetrain.TELEOP_STATES.SHOOTING);
            } else {
                drivetrain.SetTeleOpState(Drivetrain.TELEOP_STATES.NORMAL);
            }

            if (intakeDexer.bals()>=3){
                if (shootingMode == SHOOTING_MODES.RANDOM) {
                    intakeDexer.SerializeThreeMode();
                } else {
                    intakeDexer.ShootPattern(pattern);
                }

            }
        } else {
            drivetrain.SetTeleOpState(Drivetrain.TELEOP_STATES.NORMAL);
        }
        if (intakeDexer.bals()==0){

            intakeDexer.IntakeThreeMode();
        }
        telemetry.update();

    }
    public boolean isShootingDone() {
        return intakeDexer.currentState != IntakeDexer.STATES.OUTTAKE_THREE
                && intakeDexer.currentState != IntakeDexer.STATES.SHOOT_PATTERN;
    }


}

package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.Paths.GoalSidePaths;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.Paths.SIDES;
import org.firstinspires.ftc.teamcode.Subsystems.Intakedexer.IntakeDexer;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake.TurretSubsystem;

@Autonomous(name = "Goal Side Auto", group = "Main")
public class GoalSideAuto extends OpMode {

    private MainRobot mainRobot = new MainRobot();
    private GoalSidePaths goalSidePaths;
    private Follower follower;

    private enum AutoState {
        START_TO_SPIN,
        SPIN_TO_FIRST_ROW,
        INTAKE_FIRST_ROW,
        FIRST_ROW_TO_SHOOT,
        SHOOT_TO_SECOND_ROW,
        INTAKE_SECOND_ROW,
        SECOND_ROW_TO_SHOOT,
        SHOOT_TO_THIRD_ROW,
        INTAKE_THIRD_ROW,
        THIRD_ROW_TO_SHOOT,
        SHOOT_TO_LAUNCH,
        PUSH_ROBOT,
        FINISH,
        DONE
    }

    private AutoState currentState = AutoState.START_TO_SPIN;
    private boolean calledForceShoot = false;
    private double stateStartTime = 0;

    @Override
    public void init() {
        mainRobot.Init(hardwareMap, telemetry);
        follower = mainRobot.drivetrain.getFollower();
        goalSidePaths = new GoalSidePaths(follower, SIDES.BLUE);

        follower.setPose(goalSidePaths.p_Start);

        mainRobot.SetGoal(TurretSubsystem.GOALS.BLUE_GOAL);
        mainRobot.intakeDexer.Reset();
        mainRobot.outtake.Reset();

        telemetry.addLine("Status: Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.followPath(goalSidePaths.StartToSpin);
        resetStateTimer();
    }

    @Override
    public void loop() {
        mainRobot.Periodic(telemetry);

        switch (currentState) {

            case START_TO_SPIN:
                if (handleShootAndContinue(goalSidePaths.SpinToFirstRow, AutoState.SPIN_TO_FIRST_ROW)) break;
                break;

            case SPIN_TO_FIRST_ROW:
                if (!follower.isBusy()) {
                    follower.followPath(goalSidePaths.IntakeFirstRow);
                    follower.setMaxPower(0.350);
                    setState(AutoState.INTAKE_FIRST_ROW);
                }
                break;

            case INTAKE_FIRST_ROW:
                if (!follower.isBusy()) {
                    follower.followPath(goalSidePaths.FirstRowToShoot);
                    follower.setMaxPower(1);
                    setState(AutoState.FIRST_ROW_TO_SHOOT);
                }
                break;

            case FIRST_ROW_TO_SHOOT:
                if (handleShootAndContinue(goalSidePaths.ShootToSecondRow, AutoState.SHOOT_TO_SECOND_ROW)) break;
                break;

            case SHOOT_TO_SECOND_ROW:
                if (!follower.isBusy()) {
                    follower.followPath(goalSidePaths.IntakeSecondRow);
                    follower.setMaxPower(0.350);
                    setState(AutoState.INTAKE_SECOND_ROW);
                }
                break;

            case INTAKE_SECOND_ROW:
                if (!follower.isBusy()) {
                    follower.followPath(goalSidePaths.SecondRowToShoot);
                    follower.setMaxPower(1);
                    setState(AutoState.SECOND_ROW_TO_SHOOT);
                }
                break;

            case SECOND_ROW_TO_SHOOT:
                if (handleShootAndContinue(goalSidePaths.ShootToThirdRow, AutoState.SHOOT_TO_THIRD_ROW)) break;
                break;

            case SHOOT_TO_THIRD_ROW:
                if (!follower.isBusy()) {
                    follower.followPath(goalSidePaths.IntakeThirdRow);
                    follower.setMaxPower(0.350);
                    setState(AutoState.INTAKE_THIRD_ROW);
                }
                break;

            case INTAKE_THIRD_ROW:
                if (!follower.isBusy()) {
                    follower.followPath(goalSidePaths.ThirdRowToShoot);
                    follower.setMaxPower(1);
                    setState(AutoState.THIRD_ROW_TO_SHOOT);
                }
                break;

            case THIRD_ROW_TO_SHOOT:
                if (handleShootAndContinue(goalSidePaths.ShootToLaunch, AutoState.SHOOT_TO_LAUNCH)) break;
                break;

            case SHOOT_TO_LAUNCH:
                if (!follower.isBusy()) {
                    follower.followPath(goalSidePaths.PushRobot);
                    follower.setMaxPower(1);
                    setState(AutoState.PUSH_ROBOT);
                }
                break;

            case PUSH_ROBOT:
                if (!follower.isBusy()) {
                    follower.followPath(goalSidePaths.Finish);
                    setState(AutoState.FINISH);
                }
                break;

            case FINISH:
                if (!follower.isBusy()) {
                    setState(AutoState.DONE);
                }
                break;

            case DONE:
                telemetry.addLine("✅ Auto complete");
                break;
        }

        telemetry.addData("Current State", currentState);
        telemetry.addData("Follower busy", follower.isBusy());
        telemetry.addData("Shooting Done", mainRobot.isShootingDone());
        telemetry.addData("State Time (s)", timeSinceStateStart());
        telemetry.update();
    }

    /**
     * Função auxiliar que executa a lógica padrão de disparo e transição segura.
     * @param nextPath caminho seguinte ao término do disparo
     * @param nextState próximo estado após disparar
     * @return true se o disparo está em andamento ou acabou de ser completado (para evitar duplo avanço)
     */
    private boolean handleShootAndContinue(PathChain nextPath, AutoState nextState)
    { double timeout = 8.0; // segurança: se algo travar por mais de 5s, continua mesmo assim

        // Disparar uma vez
         if (!calledForceShoot && !follower.isBusy()) { mainRobot.ForceShoot();
             calledForceShoot = true;
             resetStateTimer();
             return true; }
         // Esperar disparo terminar
            if (calledForceShoot && (mainRobot.isShootingDone() || timeSinceStateStart() > timeout)) {
                follower.followPath(nextPath);
                setState(nextState);
                calledForceShoot = false;
                return true;
            }
            return false;
         }

    /** Reinicia o temporizador de estado */
    private void resetStateTimer() {
        stateStartTime = getRuntime();
    }

    /** Retorna o tempo em segundos desde o início do estado atual */
    private double timeSinceStateStart() {
        return getRuntime() - stateStartTime;
    }

    /** Atualiza o estado e reseta o timer */
    private void setState(AutoState newState) {
        currentState = newState;
        resetStateTimer();
    }
}

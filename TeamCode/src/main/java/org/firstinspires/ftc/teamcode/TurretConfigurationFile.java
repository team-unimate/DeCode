package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intakedexer.IntakeDexer;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake.TurretSubsystem;

import java.util.ArrayList;

@Config
@TeleOp
public class TurretConfigurationFile extends LinearOpMode {
    Drivetrain dt = new Drivetrain();
    ShooterSubsystem sh = new ShooterSubsystem();
    TurretSubsystem tu = new TurretSubsystem();
    IntakeDexer intakeDexer = new IntakeDexer();
    public static int velocity = 0;
    public static double kP = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {
        dt.InitSubsystems(hardwareMap, telemetry);
        sh.Init(hardwareMap, telemetry);
        tu.Init(hardwareMap, telemetry);

        dt.StartTeleOpDrive();
        dt.SetPose(new Pose(9, 9, 0));

        ArrayList<Double> distances = new ArrayList<>();
        ArrayList<Double> velocities = new ArrayList<>();
        intakeDexer.InitSubsystems(hardwareMap, telemetry);
        intakeDexer.AlternateMode();
        intakeDexer.setCanShoot(true);
        telemetry = FtcDashboard.getInstance().getTelemetry();
        waitForStart();

        while (!isStopRequested()) {

            intakeDexer.Periodic(telemetry);
            dt.Periodic(telemetry);
            sh.Periodic();

            tu.AimTo(TurretSubsystem.GOALS.BLUE_GOAL, dt.getFollower().getPose());

            tu.Periodic();

            sh.setTargetVelocity(velocity);
            sh.velocityPID.SetP(kP);
            if (gamepad1.aWasPressed()) {
                double dist = tu.GetDistanceToGoal();
                distances.add(dist);
                velocities.add((double)velocity);
            }
            telemetry.addData("Actual Velocity: ", sh.currentSpeed());
            telemetry.addData("Target Velocity", velocity);
            telemetry.addData("Distance", tu.GetDistanceToGoal());

            telemetry.addLine("=== LUT Commands to Copy ===");
            for (int i = 0; i < distances.size(); i++) {
                telemetry.addLine(String.format("lut.add(%.2f, %.2f);", distances.get(i), velocities.get(i)));
            }
            telemetry.addLine("============================");

            telemetry.update();
        }
    }
}

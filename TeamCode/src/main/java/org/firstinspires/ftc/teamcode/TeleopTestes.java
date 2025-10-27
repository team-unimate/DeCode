package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intakedexer.IntakeDexer;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake.Outtake;

@TeleOp
public class TeleopTestes extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        /*IntakeDexer intakeDexer = new IntakeDexer();
        intakeDexer.InitSubsystems(hardwareMap, telemetry);
        */
        //Outtake outtake = new Outtake();
        //outtake.InitSubsystems(hardwareMap, telemetry);
        Drivetrain dt = new Drivetrain();
        dt.InitSubsystems(hardwareMap, telemetry);
        waitForStart();
        while (!isStopRequested()){
            dt.Periodic(telemetry);
           // outtake.Periodic(telemetry);
          //  intakeDexer.Periodic(telemetry);

        }
    }
}

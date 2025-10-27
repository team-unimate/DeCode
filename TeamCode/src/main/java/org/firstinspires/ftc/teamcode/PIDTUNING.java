package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Intakedexer.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.core.controller.PIDFController;
@TeleOp
@Config
public class PIDTUNING extends LinearOpMode {
    PIDFController pid = new PIDFController(0,0,0,0);
    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;
    public static int sp = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotor encoder = hardwareMap.get(DcMotor.class, "ES");

        waitForStart();
        while (!isStopRequested()) {
            pid.SetPIDF(kp, ki, kd, 0);
            pid.setSetpoint(sp);
            int currentTicks = encoder.getCurrentPosition();

            pid.setSetpoint(sp);
            double output = pid.calculate(currentTicks);
            encoder.setPower(output);
            telemetry.addData("SP", sp);
            telemetry.addData("CP", currentTicks);
            telemetry.update();
        }
        }
}

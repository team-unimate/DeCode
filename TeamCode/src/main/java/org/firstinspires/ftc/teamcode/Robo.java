package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.controller.PIDFController;


@TeleOp
public class Robo extends LinearOpMode {


    @Override
    public void runOpMode(){
        MainRobot mainRobot = new MainRobot();
        mainRobot.Init(hardwareMap, telemetry);
        CRServo turret1 = hardwareMap.get(CRServo.class, "t1");
        CRServo turret2 = hardwareMap.get(CRServo.class, "t2");
        DcMotor encoder = hardwareMap.get(DcMotor.class, "encoder");

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        PIDFController pidf = new PIDFController(0.1, 0, 0, 0);

        waitForStart();

        while (!isStopRequested()){
            pidf.setSetpoint(pidf.getSetpoint()+(gamepad1.left_stick_y*0.001));
            turret1.setPower(pidf.calculate(encoder.getCurrentPosition()));
            turret2.setPower(pidf.calculate(encoder.getCurrentPosition()));



            mainRobot.Periodic(telemetry);
        }
    }
}

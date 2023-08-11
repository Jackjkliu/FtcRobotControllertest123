package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp (name = "TicksToInchesFinder")
public class TicksToInchesFinder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Robot bot = new Robot (hardwareMap, this);
        waitForStart();

        bot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        telemetry.addData("Left Front Ticks", bot.leftFront.getCurrentPosition());
        telemetry.addData("Left Back Ticks", bot.leftBack.getCurrentPosition());
        telemetry.addData("Right Front Ticks", bot.rightFront.getCurrentPosition());
        telemetry.addData("Right Back Ticks", bot.rightBack.getCurrentPosition());



        




        telemetry.update();
    }
}
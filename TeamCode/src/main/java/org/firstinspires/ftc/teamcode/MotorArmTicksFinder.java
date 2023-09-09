package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "MotorArmTicksFinder")
public class MotorArmTicksFinder extends LinearOpMode {
    @Override
    public void runOpMode() throws  InterruptedException{
        Robot bot = new Robot(hardwareMap, this);
        waitForStart();
        bot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(opModeIsActive()) {


            telemetry.addData("Arm motor ticks: ", bot.armMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}

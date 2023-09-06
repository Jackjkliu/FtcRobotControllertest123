package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "ServoPositionFinder")
public class SeroPositionFinder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        Robot bot = new Robot(hardwareMap, this);
        waitForStart();

        telemetry.addData("arm servo: ", bot.armServo.getPosition());
        telemetry.addData("left servo: ", bot.Lclaw.getPosition());
        telemetry.addData("right servo: ", bot.Rclaw.getPosition());

        telemetry.update();
    }

}

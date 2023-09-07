package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Auton", group = "auto")
public class Auton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        Robot bot = new Robot(hardwareMap, this);
        bot.initOpenCV();
        waitForStart();
        bot.resetAllDriveEncoders();

        int ballColor = CameraDetector.ballColor; //this detects the
        telemetry.addData("ballColor int: ", ballColor);
        telemetry.update();

        //code to program the bot to drop a pre-collected ball into the tree
        bot.moveFourBarToLow(0.6);
        bot.straight(1,12,0.6);


        //parking sequence
        if (ballColor == 1){
            bot.strafe(-1,-36,0.6);
            bot.straight(1,20,0.6);
        }
        else{
            bot.strafe(1,36,0.6);
            bot.straight(1, 20, 0.6);
        }
    }
}

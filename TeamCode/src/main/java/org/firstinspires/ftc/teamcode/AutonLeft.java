package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoLeft", group = "auto")
public class AutonLeft extends LinearOpMode {
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
        

        //parking sequence
        if (ballColor == 0){
            bot.strafe(0,-1,0.6);
            bot.straight(1,0,0.6);
        }
        else{
            bot.strafe(0,1,0.6);
            bot.straight(1, 0, 0.6);
        }
    }
}

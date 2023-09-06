package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="BumScrub")

public class Auton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot BumScrub = new Robot(hardwareMap,this);
        waitForStart();
        BumScrub.straight(1,24,0.6);
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "DriveTest")

public class Drive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap, this);

        waitForStart();
        double lx, rx, ly; // intialize variables for the gamepad
        boolean lateA = false;
        boolean lateB = false;
        boolean lateX = false;
        boolean lateY = false;
        boolean lateDdown = false;
        boolean lateDup = false;
        boolean lateDleft = false;
        boolean lateDright = false;


        while (opModeIsActive()) {
            //drive
            // set the gamepad variables
            lx = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;
            ly = gamepad1.left_stick_y;

            // arithmetic to get motor values - not scaled
            double lf = ly - rx - lx;
            double lb = ly - rx + lx;
            double rf = -ly - rx - lx;
            double rb = -ly - rx + lx;
            // scale the motor values
            double ratio;
            double max = Math.max(Math.max(Math.abs(lb), Math.abs(lf)), Math.max(Math.abs(rb), Math.abs(rf)));
            double magnitude = Math.sqrt((lx * lx) + (ly * ly) + (rx * rx));
            if (max == 0) {
                ratio = 0;
            } else {
                ratio = magnitude / max;
            }
            // sets the motor power
            if (magnitude > 0.05) {
                bot.leftFront.setPower(lf * ratio*0.58);
                bot.leftBack.setPower(lb * ratio*0.58);
                bot.rightFront.setPower(rf * ratio*0.5);
                bot.rightBack.setPower(rb * ratio*0.5);
            }
            else{
                bot.leftFront.setPower(0);
                bot.leftBack.setPower(0);
                bot.rightFront.setPower(0);
                bot.rightBack.setPower(0);
            }

            if (gamepad1.left_trigger > 0.05){
                bot.armMotor.setPower(gamepad1.left_trigger*0.6);
            }
            if (gamepad1.right_trigger > 0.05) {
                bot.armMotor.setPower(-1* gamepad1.left_trigger*0.6);
            }

            if(gamepad1.dpad_up && !lateDup){
                bot.moveFourBarToTop(0.8);
            }
            if(gamepad1.dpad_left && !lateDleft){
                bot.moveFourBarToMid(0.8);
            }
            if(gamepad1.dpad_right && !lateDright){
                bot.moveFourBarToLow(0.8);
            }
            if(gamepad1.dpad_down && !lateDdown){
                bot.resetFourBarToPickupPos(0.8);
            }

            if(gamepad1.a && !lateA) {
                bot.armMotor.setTargetPosition(100);
            }
            if(gamepad1.left_bumper){
                bot.closeClaw();
            }
            if(gamepad1.right_bumper){
                bot.openClaw();
            }
            if(gamepad1.x && !lateX){
                bot.moveArmToStar(0.8);
            }
            if(gamepad1.y && !lateY){
                bot.stockingPreset(0.8);
            }
            if(gamepad1.b && !lateB){
                bot.junctionPreset(0.8);
            }


            telemetry.addData("Test: ", bot.armMotor.getCurrentPosition());
//            telemetry.addData("RF: ", bot.rightFront.getPower());
//            telemetry.addData("RB: ", bot.rightBack.getPower());
//            telemetry.addData("LF: ", bot.leftFront.getPower());
//            telemetry.addData("LB: ", bot.leftBack.getPower());
//
//
//            //bottom: 0: top around 4181  mid:2985 bottom:1679
//            telemetry.addData("y val:", gamepad1.left_stick_y);
//            telemetry.addData("lb:", lb);
//            telemetry.addData("lf", lf);
//            telemetry.addData("rf", rf);
//            telemetry.addData("rb", rb);


            telemetry.update();
            lateA = gamepad1.a;
            lateB = gamepad1.b;
            lateX = gamepad1.x;
            lateY = gamepad1.y;
            lateDdown = gamepad1.dpad_down;
            lateDup = gamepad1.dpad_up;
            lateDleft = gamepad1.dpad_left;
            lateDright = gamepad1.dpad_right;

        }
    }
}

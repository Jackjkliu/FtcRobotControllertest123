ampackage org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name="Batman")

public class TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot Batman = new Robot(hardwareMap,this);
        waitForStart();

        double lx, rx, ly;

        while (opModeIsActive()){
            lx = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;
            ly = gamepad1.left_stick_y;

            double lf = ly + lx + rx;
            double lb = ly - lx + rx;
            double rf = ly - lx - rx;
            double rb = ly + lx - rx;

            double ratio;
            double max = Math.max(Math.max(Math.abs(lb),Math.abs(lf)),Math.max(Math.abs(rb),Math.abs(rf)));
            double magnitude = Math.sqrt((lx * lx) + (ly * ly) + (rx * rx));
            if (max == 0) {
                ratio = 0;
            }
            else {
                ratio = magnitude / max;
            }

            if (magnitude > 0.05) { //0.05 is deadzone, if less we just don't do anything and assume its error
                Batman.leftFront.setPower(lf * ratio);
                Batman.leftBack.setPower(lb * ratio);
                Batman.rightFront.setPower(rf * ratio);
                Batman.rightBack.setPower(rb * ratio);
            }
            else {
                Batman.leftFront.setPower(0);
                Batman.leftBack.setPower(0);
                Batman.rightFront.setPower(0);
                Batman.rightBack.setPower(0);
            }
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "EncoderTest")
public class EncoderTest extends LinearOpMode {
//    FtcDashboard dashboard;
    DcMotorEx TestMotor;
    @Override
    public void runOpMode() throws InterruptedException {
//        dashboard = FtcDashboard.getInstance();
        TestMotor = hardwareMap.get(DcMotorEx.class, "motor");
//        Robot bot = new Robot(hardwareMap, this);
//        telemetry = dashboard.getTelemetry();

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("EncoderTest: ", TestMotor.getCurrentPosition());
            telemetry.update();
        }

    }
}

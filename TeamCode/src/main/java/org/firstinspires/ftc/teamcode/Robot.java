package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot{

    public DcMotorEx leftBack, leftFront, rightBack, rightFront;
    public BNO055IMU imu;
    Orientation currentAngle;
    double ticksToInches;

    LinearOpMode linearOpMode;
    HardwareMap hardwareMap;
    Telemetry telemetry;


    public Robot(HardwareMap hardwareMap, LinearOpMode linearOpMode) {
        leftBack = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightRear");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        currentAngle = imu.getAngularOrientation();
        this.linearOpMode = linearOpMode;
        this.hardwareMap = hardwareMap;

    }
    public double angleCompare(double angle1, double angle2){
        if(angle1 > angle2){
            return angle1-angle2;
        }
        else if(angle2 > angle1){
            return angle2-angle1;
        }
        else {
            return 0;
        }

    }
    public void straight(double direction, double distance, double speed){ //method for forward/backward
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double ticksmoved = 0;
        double tickstoinches = 3.14;
        while(ticksmoved * tickstoinches < distance){
            leftFront.setPower(speed*direction);
            rightFront.setPower(speed*direction);
            leftBack.setPower(speed*direction);
            rightBack.setPower(speed*direction);
            ticksmoved = Math.abs(leftFront.getCurrentPosition());
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void turn(double angle, double speed, double direction){//method for rotating
        while(Math.abs(angleCompare(imu.getAngularOrientation().firstAngle, angle)) > 0){
            leftFront.setPower(speed*direction);
            rightFront.setPower(-speed*direction);
            leftBack.setPower(speed*direction);
            rightBack.setPower(-speed*direction);
            //imu.getAngularOrientation().firstAngle;
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    public void strafe(double distance, int direction, double speed){ //method for strafing
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double ticksmoved = 0;
        double tickstoinches = 3.14;
        while(ticksmoved * tickstoinches < distance){
            leftFront.setPower(speed * direction);
            rightFront.setPower(speed * direction*-1);
            leftBack.setPower(speed * direction*-1);
            rightBack.setPower(speed * direction);
            ticksmoved = Math.abs(leftFront.getCurrentPosition());

        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }




}








//method for four bar

//method for claw

//method for servo

//method(s) for camera
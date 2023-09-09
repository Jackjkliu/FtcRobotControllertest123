package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
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

    public DcMotorEx leftBack, leftFront, rightBack, rightFront, armMotor;
    public Servo armServo, Lclaw, Rclaw;
    public OpenCvWebcam webcam;
    public BNO055IMU imu;

    Orientation currentAngle;
    double ticksToInches;
    double ticksToInchesStrafe;
    boolean clawIsClosed;

    LinearOpMode linearOpMode;
    HardwareMap hardwareMap;
    Telemetry telemetry;


    public Robot(HardwareMap hardwareMap, LinearOpMode linearOpMode) {
        leftBack = hardwareMap.get(DcMotorEx.class, "lb");
        rightBack = hardwareMap.get(DcMotorEx.class, "rb");
        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        armMotor = hardwareMap.get(DcMotorEx.class, "fourBar");
        armServo = hardwareMap.get(Servo.class, "arm");
        Lclaw = hardwareMap.get(Servo.class, "leftClaw");
        Rclaw = hardwareMap.get(Servo.class,"rightClaw");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        currentAngle = imu.getAngularOrientation();
        this.linearOpMode = linearOpMode;
        this.hardwareMap = hardwareMap;
        ticksToInches = 20;//change later
        ticksToInchesStrafe = 27.5;
        clawIsClosed = false;

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //added this later, hopefully it doesn't break anything
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double ticksmoved = 0;
        while(ticksmoved * ticksToInches < distance){
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
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void turn(double angle, double speed, double direction){//method for rotating
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //added this later, hopefully it doesn't break anything
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void strafe(int direction, double distance, double speed){ //method for strafing
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //added this later, hopefully it doesn't break anything
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double ticksmoved = 0;
        while(ticksmoved * ticksToInchesStrafe < distance){
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
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void initOpenCV() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"),
                cameraMonitorViewId);
        webcam.setPipeline(new CameraDetector());
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        OpenCvWebcam finalWebcam = webcam;
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                finalWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }


        });
    }




    //below are the methods added for Auton and Teleop
    public void moveFourBarArm(double desiredTickPosition, double speed){
        //speed is any value type (negative does not matter)
//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int buffer = 0;//it may be the case where the bot will try to oscillate the motor to reach exactly the desiredTickPosition. The buffer adds some leniency.
        while(armMotor.getCurrentPosition() < desiredTickPosition - buffer || armMotor.getCurrentPosition() > desiredTickPosition + buffer){
            if(armMotor.getCurrentPosition() < desiredTickPosition){//this line may need to be "if(armMotor.getCurrentPosition() < desiredTickPosition - buffer)" if there is an error
                armMotor.setPower(Math.abs(speed));
            }
            else if(armMotor.getCurrentPosition() > desiredTickPosition) {//this line may need to be "if(armMotor.getCurrentPosition() > desiredTickPosition + buffer)" if there is an error
                armMotor.setPower(-1 * Math.abs(speed));
            }
        }
    }
    public void moveArmServo(double position){
        armServo.setPosition(position);
    }

    //followed 3 methods are related to the tree
    public void moveFourBarToTop(double speed){
        int encoderPosition = 200;//find the encoder value for the top level
        armMotor.setPower(0.6);
        armMotor.setTargetPosition(encoderPosition);
        double servoPosition = 0;
        moveArmServo(servoPosition);
    }
    public void moveFourBarToMid(double speed){
        int encoderPosition = 170;//find the encoder value for the mid level
        armMotor.setPower(0.6);
        armMotor.setTargetPosition(encoderPosition);
        double servoPosition = 0.5;
        moveArmServo(servoPosition);
    }
    public void moveFourBarToLow(double speed){
        int encoderPosition = 160;//find the encoder value for the low level
        armMotor.setPower(0.6);
        armMotor.setTargetPosition(encoderPosition);
        double servoPosition = 0.6;
        moveArmServo(servoPosition);
    }
    public void resetFourBarToPickupPos(double speed){
        int encoderPosition = 0;//find the encoder value for the pickup level
        armMotor.setPower(0.3);
        armMotor.setTargetPosition(encoderPosition);
        double servoPosition = 0.75;
        moveArmServo(servoPosition);
    }
    public void closeClaw(){
        Lclaw.setPosition(0.74);
        Rclaw.setPosition(0.06);
    }
    public void openClaw(){
        Lclaw.setPosition(0.4);
        Rclaw.setPosition(0.4);
    }
    public void moveArmToStar(double speed){
        double encoderPosition = 200;//find the encoder value for the star
        moveFourBarArm(encoderPosition,speed);
        double servoPosition = 0;
        moveArmServo(servoPosition);
    }
    public void junctionPreset(double speed){
        double encoderPosition = 200;//find the encoder position to bring the arm up to the low junction
        moveFourBarArm(encoderPosition, speed);
        double servoPosition = 0;
        moveArmServo(servoPosition);
    }
    public void stockingPreset(double speed){
        double encoderPosition = 77; //findthe encoder position to being the arm up to the stocking
        moveFourBarArm(encoderPosition, speed);
        double servoPosition = 0;
        moveArmServo(servoPosition);
    }
    public void resetAllDriveEncoders(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}




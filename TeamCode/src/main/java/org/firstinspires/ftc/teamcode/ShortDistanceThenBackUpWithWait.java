package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.annotation.Target;
import java.util.Set;

/**
 * Created by Trevor on 11/6/2016.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "8 inch -> Shoot 2 -> BackUp", group = "6994 Bot")
public class ShortDistanceThenBackUpWithWait extends LinearHardwareMap {
    public float Linearlasterror;
    int InitialTheta = 30;
    double HypotenuseLength = 50;
    double HypotenuseDriveTime = 3;
    ElapsedTime runtime = new ElapsedTime();
    double LeftPower = 0;
    double RightPower = .25;
    double ReverseLeftPower = .25;
    double ReverseRightPower = 0;
    int TimeOut=3;
    double minPower=.25;
    double Runtime=3;


    @Override
    public void runOpMode() throws InterruptedException {
        FrontLeft = hardwareMap.dcMotor.get(frontLeftMotor);
        FrontRight = hardwareMap.dcMotor.get(frontRightMotor);
        BackLeft = hardwareMap.dcMotor.get(backLeftMotor);
        BackRight = hardwareMap.dcMotor.get(backRightMotor);
        Gyro = hardwareMap.gyroSensor.get(gyroSensor);
        //ButtonPusherLeft = hardwareMap.servo.get(buttonPusherLeft);
        //ButtonPusherRight = hardwareMap.servo.get(buttonPusherRight);
        BeaconColorSensor = hardwareMap.colorSensor.get(beaconColorSensor);
        WhiteLineFinder = hardwareMap.opticalDistanceSensor.get(whiteLineFinder);
        BeaconColorSensor = hardwareMap.colorSensor.get(beaconColorSensor);
        Catapult = hardwareMap.dcMotor.get(catapult);
        CatapultStop = hardwareMap.touchSensor.get(catapultStop);
        BallCollection = hardwareMap.dcMotor.get(ballCollection);
        BallControl = hardwareMap.servo.get(ballControll);
        BallControl.setPosition(ballControlStartPosition);
        SideRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, sideRangeSensor);
        FrontRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, frontRangeSensor);
        BackRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, backRangeSensor);
//
        //FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        setPower(0,0,0,0);
        SetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();

        sleep(2000);


        /*while (Gyro.isCalibrating() && opModeIsActive()) {
            telemetry.addData(">", "Calibrating Gyro");
            telemetry.update();
            idle();
            sleep(50);
            telemetry.addData(">", "Ready!");
            telemetry.addData(">", "Hey Jason, Try not to Fuck up");
            telemetry.update();
        }*/
        waitForStart();
        if(opModeIsActive())
        {
            //BallControl.setPosition(ballControlStartPosition);
            Drive(.25,8,0,4,false);
            while(!CatapultStop.isPressed()){Catapult.setPower(1);}Catapult.setPower(0);
            sleep(500);
            BallControl.setPosition(0);
            while (CatapultStop.isPressed()){Catapult.setPower(1);}Catapult.setPower(0);
            sleep(500);
            while (!CatapultStop.isPressed()) {Catapult.setPower(1);}
            Catapult.setPower(0);
            BallControl.setPosition(1);
            sleep(500);
            BallCollection.setPower(1);
            sleep(3000);
            BallCollection.setPower(0);
            BallControl.setPosition(0);
            while(!CatapultStop.isPressed()){Catapult.setPower(1);}Catapult.setPower(0);
            sleep(500);
            while (CatapultStop.isPressed()){Catapult.setPower(1);}Catapult.setPower(0);
            sleep(500);
            while (!CatapultStop.isPressed()) {Catapult.setPower(1);}
            //Catapult.setPower(0);

            //sleep(4000);
            Catapult.setPower(0);
            setPower(0, 0, 0, 0);
            setPower(0, 0, 0, 0);
            Catapult.setPower(0);
            BallCollection.setPower(0);
            requestOpModeStop();

        }
        else
        {
            BallControl.setPosition(ballControlStartPosition);
            BallCollection.setPower(0);
            setPower(0,0,0,0);
            Catapult.setPower(0);

        }}
}


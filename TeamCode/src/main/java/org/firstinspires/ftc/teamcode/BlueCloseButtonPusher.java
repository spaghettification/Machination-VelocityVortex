package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Trevor on 11/21/2016.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BlueCloseButtonPusher", group = "6994 Bot")
@Disabled
public class BlueCloseButtonPusher extends LinearHardwareMap {
    @Override
    public void runOpMode(){
        AutonomousHardwareMap();
        StopAllMotors();
        InitializeServoPositions();
        Calibrate();
        sleep(50);


        waitForStart();
        Gyro.calibrate();
        while (Gyro.isCalibrating()){sleep(500);}
        while(opModeIsActive()&&!isStopRequested()&& !Gyro.isCalibrating()) {
            Turn(.25,0,45,"clockwise");
            Drive(.2, 45, 0, 5, false);
            Turn(0,.2,5,"counterclockwise");
            FindWhiteLine(WhiteLineFinder,.2);
            ButtonPusherArm.setPosition(buttonPusherEngage);
            ButtonPusher.setPosition(.9);
            sleep(500);
            if (BeaconColorSensor.blue()>BeaconColorSensor.red())
            {ButtonPusher.setPosition(ButtonPusher.getPosition()+.1);}
            else ButtonPusher.setPosition(buttonPusherRight);
            sleep(10000);
            telemetry.addData("blue",BeaconColorSensor.blue());
            telemetry.addData("red",BeaconColorSensor.red());
            telemetry.update();
            requestOpModeStop();




            //////////////////////////////////////////////////////////////////////New Path
            /*Drive(.375, 66.5, 0, 5, false);
            // shootParticle(1);
            Turn(.175, 87, false, "clockwise");
            Drive(.225,44,90,6,false);
            //DriveToWall(5,.25);
            sleep(5000);
            requestOpModeStop();*/
          /*  ButtonPush("Blue");
            setPower(0, 0, 0, 0);
            Drive(.325, -8, 90, 2, false);
            Turn(.125, 0, false, "CounterClockwise");
            Drive(.25, 40, getIntegratedZValue(), 5, false);
            FindWhiteLine(WhiteLineFinder, .175);
            Turn(.125, 90, false, "Clockwise");
            ButtonPush("Blue");
            sleep(500);
            Drive(.125, -6, 90, 3, false);
            Turn(.125, 0, false, "CounterClockwise");
            sleep(500);
            Drive(.125, -8, getIntegratedZValue(), 5, false);
            Turn(.125, 150, false, "clockwise");
            //l shootParticle(1);
            Drive(.25, 50, getIntegratedZValue(), 5, false);
*/

        }
    }

}


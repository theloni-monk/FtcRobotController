package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Distance PID Test", group="Linear Opmode")
public class DistancePIDTestOp extends AutoBot {

    @Override
    public void runOpMode() {
        initDriveOp();
        initTracking();
        initPIDs();
        // Set up our telemetry dashboard
        composeTelemetry();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // Loop and update the dashboard
        while (opModeIsActive()) {
            telemetry.update();
            executeControllerDriveLogic();
            if(gamepad1.x && debounced) {
                driveDist(1, DistanceUnit.METER); // move forward in y axis 1m
                debounce.run();}
        }
    }


}

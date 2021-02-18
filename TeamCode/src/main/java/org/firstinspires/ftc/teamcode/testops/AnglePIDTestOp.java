package org.firstinspires.ftc.teamcode.testops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.botfunctionality.AutoBot;
import org.firstinspires.ftc.teamcode.botfunctionality.PositionTrackerBot;
import org.firstinspires.ftc.teamcode.utils.PIDController;

@TeleOp(name="Angle PID Test", group="Linear Opmode")
public class AnglePIDTestOp extends AutoBot {


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
            if (gamepad1.y && this.debounced) {
                rotate(90, triggerPower);
                debounce.run();
            }
            if (gamepad1.x && this.debounced) {
                rotate(-90, triggerPower);
                debounce.run();
            }
        }
    }
}
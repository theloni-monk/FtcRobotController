package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Distance PID Test", group="Linear Opmode")
public class DistancePID extends PositionTrackerBot {
    PIDController straightDrivePID;
    PIDController distanceDrivePID;

    Orientation lastAngles = new Orientation();
    double globalAngle, angleCorrectionPow = .30, angleCorrection, distanceCorrection;

    @Override
    public void runOpMode() {
        initDriveOp();
        initTracking();
        straightDrivePID = new PIDController(.05, 0, 0);
        distanceDrivePID = new PIDController(0.05,0,0);//TODO: tune
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

    protected void driveDist(double dist, DistanceUnit unit){
        double initLoc = this.rNav.getPosition().toUnit(unit).y;

        straightDrivePID.setSetpoint(0);
        straightDrivePID.setOutputRange(0, angleCorrectionPow);
        straightDrivePID.setInputRange(-90, 90);
        straightDrivePID.enable();

        // Use PID with position integrator to drive a distance
        distanceDrivePID.reset();
        distanceDrivePID.setSetpoint(initLoc + dist);
        distanceDrivePID.setInputRange(0, initLoc + dist + 0.5);
        distanceDrivePID.setOutputRange(0, 1);
        distanceDrivePID.setTolerance(1);
        distanceDrivePID.enable();


        do {
            // Use PID with imu input to drive in a straight line.
            angleCorrection = straightDrivePID.performPID(getAngle());

            double yLoc = this.rNav.getPosition().toUnit(unit).y;
            distanceCorrection = distanceDrivePID.performPID(yLoc); // TODO: make it an arbitrary distance vector

            // set power levels.
            runLeftMotorPow(distanceCorrection - angleCorrection);
            runRightMotorPow(distanceCorrection + angleCorrection);
        } while (opModeIsActive() && !distanceDrivePID.onTarget());

        // brake at end
        runLeftMotorPow(0);
        runRightMotorPow(0);
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

}

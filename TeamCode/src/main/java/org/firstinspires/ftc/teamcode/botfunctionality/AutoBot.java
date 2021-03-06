package org.firstinspires.ftc.teamcode.botfunctionality;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.NavUtil;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.botfunctionality.PositionTrackerBot;
import org.firstinspires.ftc.teamcode.utils.PIDController;

abstract public class AutoBot extends PositionTrackerBot {
    PIDController straightDrivePID;
    PIDController distanceDrivePID;
    PIDController pidRotate;

    Orientation lastAngles = new Orientation();
    double globalAngle, rotation, angleCorrectionPow = .30, angleCorrection, distanceCorrection;

    /**
     * Extend this class to access pid rotation and distance movement
     */
    public AutoBot(){}

    protected void initPIDs(){
        pidRotate = new PIDController(.003, .00003, 0);
        straightDrivePID = new PIDController(.05, 0, 0);
        distanceDrivePID = new PIDController(0.5,0,0.2);//getDouble("P"),getDouble("I"),getDouble("D"));//TODO: tune
    }

    public void initAutoBot(){
        initDriveOp();
        initTracking();
        initPIDs();
    }

    @Override
    protected void composeTelemetry() {
        super.composeTelemetry();
        telemetry.addLine().addData("PID Vals",new Func<String>() {
            @Override public String value() {
                return "P: %s, I: %s, D: %s".format(String.valueOf(distanceDrivePID.getP()),String.valueOf(distanceDrivePID.getI()),String.valueOf(distanceDrivePID.getD()));
            }
        });
    }

    /**
     * moves the robot a specified distance in the y direction TODO: arbitrary dir vect
     * @param dist
     * @param unit
     */
    protected void driveDist(double dist, DistanceUnit unit) {

        Position initLoc = this.rNav.getPosition().toUnit(unit);
        resetAngle();
        straightDrivePID.setSetpoint(0);
        straightDrivePID.setOutputRange(0, angleCorrectionPow);
        straightDrivePID.setInputRange(-90, 90);
        straightDrivePID.enable();

        // Use PID with position integrator to drive a distance
        distanceDrivePID.reset();
        //distanceDrivePID.setPID(getDouble("P"),getDouble("I"),getDouble("D"));
        distanceDrivePID.setSetpoint(dist);
        distanceDrivePID.setInputRange(0, dist);
        distanceDrivePID.setOutputRange(-1, 1);
        distanceDrivePID.setTolerance(5);
        distanceDrivePID.enable();


        do {
            // Use PID with imu input to drive in a straight line.
            angleCorrection = straightDrivePID.performPID(getAngle());

            Position displacementVect = NavUtil.minus(initLoc,this.rNav.getPosition().toUnit(unit));
            double displacementMag = Math.sqrt(displacementVect.x*displacementVect.x + (displacementVect.y * displacementVect.y) + (displacementVect.z * displacementVect.z));
            distanceCorrection = distanceDrivePID.performPID(displacementMag);

            // set power levels.
            runLeftMotorPow(distanceCorrection - angleCorrection);
            runRightMotorPow(distanceCorrection + angleCorrection);

            telemetry.update();
        } while (opModeIsActive() && !distanceDrivePID.onTarget());

        // brake at end
        runLeftMotorPow(0);
        runRightMotorPow(0);
    }

    protected void driveTo(Position toPos){
        Position displacementVect = NavUtil.minus(this.rNav.getPosition().toUnit(toPos.unit), toPos);
        double displacementMag = Math.sqrt(displacementVect.x*displacementVect.x + (displacementVect.y * displacementVect.y) + (displacementVect.z * displacementVect.z));
        double radAngle = Math.atan2(displacementVect.y, displacementVect.x);
        rotate((int) AngleUnit.DEGREES.fromRadians(radAngle), 1);
        driveDist(displacementMag, toPos.unit);
    }



    //TODO: WRITEME absolute rotation
    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    protected void rotate(int degrees, double power) {
        // restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0)
            {
                runLeftMotorPow(power);
                runRightMotorPow(-power);
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                runLeftMotorPow(-power);
                runRightMotorPow(power);
                telemetry.update();
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                runLeftMotorPow(-power);
                runRightMotorPow(power);
                telemetry.update();
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        runRightMotorPow(0);
        runLeftMotorPow(0);

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
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

package org.firstinspires.ftc.teamcode.botfunctionality;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.botfunctionality.HolonomicDriveBot;
import org.firstinspires.ftc.teamcode.utils.RobotNavigation;

import java.util.Locale;

abstract public class PositionTrackerBot extends HolonomicDriveBot {
    // The IMU sensor object
    protected BNO055IMU imu;

    private DistanceSensor frontRangeSensor;
    private DistanceSensor sideRangeSensor;

    RobotNavigation rNav;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    public PositionTrackerBot(){}

    public void initTracking(){
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = (BNO055IMUImpl) hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        frontRangeSensor = hardwareMap.get(DistanceSensor.class, "range_front");
        sideRangeSensor = hardwareMap.get(DistanceSensor.class, "range_side");


        RobotNavigation dummyNav = new RobotNavigation();
        rNav = new RobotNavigation(imu, dummyNav.makeIntegrator(l1,r1,COUNTS_PER_MOTOR_REV));
        rNav.startTracking(200, parameters, new Position(), new Velocity()); // start is origin, 5hz update just for testing

    }

    public void initPosTrackerBot(){
        initDriveOp();
        initTracking();
    }
    protected void composeTelemetry() {
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //gravity  = imu.getGravity();
        }
        });

        //telemetry.addLine().addData("Motors", "Speeds: left (%.2f)m/s, right (%.2f)m/s", 2 * this.leftAngVel * (WHEEL_RAD * WHEEL_RAD), 2 * this.rightAngVel * (WHEEL_RAD * WHEEL_RAD));
        telemetry.addLine().addData("position: ", new Func<String>() {
            @Override public String value() {
                return rNav.getPosition().toString();
            }
        }).addData("velocity: ", new Func<String>() {
            @Override public String value() {
                return rNav.getVelocity().toString();
            }
        }).addData("acceleration: ", new Func<String>() {
            @Override
            public String value() {
                return rNav.getAcceleration().toString();
            }
        });

        telemetry.addLine()
                .addData("heading: ", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                });

        telemetry.addLine().addData("Range Sensors: ", new Func<String>() {
            @Override public String value() {
                return "front range sensor: " + Double.toString(frontRangeSensor.getDistance(DistanceUnit.MM))
                        + " side range sensor: " + Double.toString(sideRangeSensor.getDistance(DistanceUnit.MM));
            }
        });
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}

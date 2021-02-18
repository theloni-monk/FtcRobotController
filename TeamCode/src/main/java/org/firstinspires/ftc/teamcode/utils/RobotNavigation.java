package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.NavUtil;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Date;
import java.util.concurrent.CancellationException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.NavUtil.plus;
import static org.firstinspires.ftc.robotcore.external.navigation.NavUtil.scale;

//TODO make actually smart with kalman filter
public class RobotNavigation {
    BNO055IMU imu;
    public SmartIntegrator integrator;
    public ExecutorService navManager;

    /**
     * @param bno: must be already initialized
     * @param smartIntegrator
     */
    public RobotNavigation(BNO055IMU bno, SmartIntegrator smartIntegrator){
        this.imu = bno;
        this.integrator = smartIntegrator;
        this.navManager = ThreadPool.newSingleThreadExecutor("imu integration");
    }

    public Position getPosition() {
        return this.integrator.getPosition();
    }

    public Velocity getVelocity() {
        return this.integrator.getVelocity();
    }

    public Acceleration getAcceleration() {
        return this.integrator.getAcceleration();
    }

    /**
     * This will start tracking in relative mode, make sure to have robot at origin when starting
     * @param msPoll
     */
    public void startTracking(int msPoll, BNO055IMU.Parameters parameters, Position initialPosition, Velocity initialVelocity){
        this.integrator.initialize(parameters, initialPosition, initialVelocity);
        this.navManager.execute(new NavManager(msPoll));
    }

    public void stopTracking() {
        if (this.navManager != null) {
                this.navManager.shutdownNow();
                ThreadPool.awaitTerminationOrExitApplication(this.navManager, 10L, TimeUnit.SECONDS, "imu integration", "unresponsive user acceleration code");
                this.navManager = null;
            }
    }

    public SmartIntegrator makeIntegrator(DcMotorImplEx lDrive, DcMotorImplEx rDrive, double counts_per_motor_rev){
        return new SmartIntegrator(lDrive,rDrive,counts_per_motor_rev);
    }

    class SmartIntegrator implements BNO055IMU.AccelerationIntegrator {
        BNO055IMU.Parameters parameters = null;
        Position position = new Position();
        Velocity velocity = new Velocity();
        Acceleration acceleration = null;

        Date date;
        //long timeLastCorrected;
        final double angleEpsilon = 3;//degrees
        final double maxDistRange = 65535; //millimeters

        DcMotorImplEx lDrive;
        int lDrivePrevCount;
        DcMotorImplEx rDrive;
        int rDrivePrevCount;
        //private DistanceSensor sideRange;
        //private DistanceSensor frontRange;
        double COUNTS_PER_MOTOR_REV;
        final double imuWeight = 0.05;

        //FIXME: use motor params and use encoders to estimate pos
        SmartIntegrator(DcMotorImplEx lDrive, DcMotorImplEx rDrive, double counts_per_motor_rev) {
            date = new Date();
            this.lDrive = lDrive;
            this.rDrive = rDrive;
            COUNTS_PER_MOTOR_REV = counts_per_motor_rev;
        }

        /** range correction - deprecated
         * Corrects the internal position variable from range sensors using knowledge of field dimensions and orientation
         * also corrects internal velocity if time between calls is small enough
        public void correctFromRangeSensors(Orientation angles) {
            //Orientation angles = BNO055IMUImpl.this //TODO: fuck me
            //TODO: implement this outside of rev api
            long timeDelta = date.getTime() - timeLastCorrected;
            Position prevPos = this.position;

            double sideDist = sideRange.getDistance(DistanceUnit.MM);
            double frontDist = frontRange.getDistance(DistanceUnit.MM);
            double theta = angles.firstAngle; // heading

            //WRITEME: find wall config based on orientation then calculate position from trig relative to wall
            if (frontDist < maxDistRange) {
                //141inch long field
                if (withinEpsilon(theta, 0)) {
                    this.position.y = 3581.4 - (frontDist );
                }
                if (withinEpsilon(theta, 180) || withinEpsilon(theta, -180)) {
                    this.position.y = frontDist ;
                }
            }
            if (sideDist < maxDistRange) {
                if (withinEpsilon(theta, 0)) {
                    this.position.x = 3581.4 - (sideDist );
                }
                if (withinEpsilon(theta, 180) || withinEpsilon(theta, -180)) {
                    this.position.x = sideDist ;
                }
            }

//            if (timeDelta < 10) { //update vel from pos derivative if delta t is dt
//                this.velocity.xVeloc = (this.position.x - prevPos.x) / (timeDelta / 1000);
//                this.velocity.yVeloc = (this.position.y - prevPos.y) / (timeDelta / 1000);
//            }

            return;
        }

         */


        public Position getPosition() {
            return this.position;
        }

        public Velocity getVelocity() {
            return this.velocity;
        }

        public Acceleration getAcceleration() {
            return this.acceleration;
        }

        public void initialize(BNO055IMU.Parameters parameters, Position initialPosition, Velocity initialVelocity) {
            this.parameters = parameters;
            this.position = initialPosition != null ? initialPosition : this.position;
            this.velocity = initialVelocity != null ? initialVelocity : this.velocity;
            this.acceleration = null;
        }

        public void update(Acceleration linearAcceleration) {
            //Naive integration
            int lCount = lDrive.getCurrentPosition();
            int lDiff = lCount - lDrivePrevCount;
            lDrivePrevCount = lCount;

            int rCount = rDrive.getCurrentPosition();
            int rDiff = rCount - rDrivePrevCount;
            rDrivePrevCount = rCount;

            double meanDiffMM = (rDiff*COUNTS_PER_MOTOR_REV + (lDiff * COUNTS_PER_MOTOR_REV))/2;
            double xyAngle = RobotNavigation.this.imu.getAngularOrientation(AxesReference.INTRINSIC, ZYX, AngleUnit.RADIANS).firstAngle;
            //double diffAngle = Math.atan() //TODO: WRITEME: angle of normal to vector between yldiff and yrdiff;
            double newXDelta = Math.cos(xyAngle) * meanDiffMM;
            double newYDelta = Math.sin(xyAngle) * meanDiffMM;
            Position encoderCorrection = new Position(DistanceUnit.MM, newXDelta, newYDelta, this.position.z, 0);

            Position imuCorrection = new Position();
            if (linearAcceleration.acquisitionTime != 0L) {
                if (this.acceleration != null) {
                    Acceleration accelPrev = this.acceleration;
                    Velocity velocityPrev = this.velocity;
                    this.acceleration = linearAcceleration;
                    if (accelPrev.acquisitionTime != 0L) {
                        Velocity deltaVelocity = NavUtil.meanIntegrate(this.acceleration, accelPrev);
                        this.velocity = NavUtil.plus(this.velocity, deltaVelocity);
                    }
                    if (velocityPrev.acquisitionTime != 0L) {
                        imuCorrection = NavUtil.meanIntegrate(this.velocity, velocityPrev);
                    }

                } else {
                    this.acceleration = linearAcceleration;
                }
            }
            //get new correction from weighted avg
            Position posDelta = plus(scale(encoderCorrection, 1 - imuWeight),scale(imuCorrection, imuWeight));
            this.position = NavUtil.plus(this.position, posDelta);
        }


        private boolean withinEpsilon(double value, double target) {
            return value < target + angleEpsilon && value > target - angleEpsilon;
        }

    }

    class NavManager implements Runnable {
        protected final int msPollInterval;
        protected static final long nsPerMs = 1000000L;

        NavManager(int msPollInterval) {
            this.msPollInterval = msPollInterval;
        }

        public void run() {
            try {

                while(!Thread.currentThread().isInterrupted()) {

                    Acceleration linearAcceleration = RobotNavigation.this.imu.getLinearAcceleration();
                    RobotNavigation.this.integrator.update(linearAcceleration);


                    if (this.msPollInterval > 0) {
                        long msSoFar = (System.nanoTime() - linearAcceleration.acquisitionTime) / nsPerMs;
                        long msReadFudge = 5L;
                        Thread.sleep(Math.max(0L, (long)this.msPollInterval - msSoFar - msReadFudge));
                    } else {
                        Thread.yield();
                    }
                }

            } catch (CancellationException | InterruptedException var7) {
            }
        }
    }


}

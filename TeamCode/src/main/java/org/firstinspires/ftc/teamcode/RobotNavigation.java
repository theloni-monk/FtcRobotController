package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.NavUtil;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Date;
import java.util.concurrent.CancellationException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.robotcore.external.navigation.NavUtil.plus;

//TODO make actually smart with kalman filter
public class RobotNavigation {
    BNO055IMU imu;
    public SmartIntegrator integrator;
    protected ExecutorService navManager;

    /**
     * @param bno: must be already initialized
     * @param smartIntegrator
     */
    RobotNavigation(BNO055IMU bno, SmartIntegrator smartIntegrator){
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

    static class SmartIntegrator implements BNO055IMU.AccelerationIntegrator {
        BNO055IMU.Parameters parameters = null;
        Position position = new Position();
        Velocity velocity = new Velocity();
        Acceleration acceleration = null;

        Date date;
        long timeLastCorrected;
        final double angleEpsilon = 3;//degrees
        final double maxDistRange = 65535; //millimeters
        private DistanceSensor sideRange;
        private DistanceSensor frontRange;

        SmartIntegrator(DistanceSensor fRange, DistanceSensor sRange) {
            date = new Date();
            frontRange = fRange;
            sideRange = sRange;
        }

        /**
         * Corrects the internal position variable from range sensors using knowledge of field dimensions and orientation
         * also corrects internal velocity if time between calls is small enough
         * TODO: write in fudge factors for location of range sensors on robot
         *
         * @param angles
         */
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
                    this.position.y = frontDist / 1000;
                }
                if (withinEpsilon(theta, 180) || withinEpsilon(theta, -180)) {
                    this.position.y = 3.5814 - (frontDist / 1000);
                }
            }
            if (sideDist < maxDistRange) {
                if (withinEpsilon(theta, 0)) {
                    this.position.x = sideDist / 1000;
                }
                if (withinEpsilon(theta, 180) || withinEpsilon(theta, -180)) {
                    this.position.x = 3.5814 - (sideDist / 1000);
                }
            }

            if (timeDelta < 10) { //update vel from pos derivative if delta t is dt
                this.velocity.xVeloc = (this.position.x - prevPos.x) / (timeDelta / 1000);
                this.velocity.yVeloc = (this.position.y - prevPos.y) / (timeDelta / 1000);
            }

            return;
        }


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
            if (linearAcceleration.acquisitionTime != 0L) {
                if (this.acceleration != null) {
                    Acceleration accelPrev = this.acceleration;
                    Velocity velocityPrev = this.velocity;
                    this.acceleration = linearAcceleration;
                    if (accelPrev.acquisitionTime != 0L) {
                        Velocity deltaVelocity = NavUtil.meanIntegrate(this.acceleration, accelPrev);
                        this.velocity = plus(this.velocity, deltaVelocity);
                    }

                    if (velocityPrev.acquisitionTime != 0L) {
                        Position deltaPosition = NavUtil.meanIntegrate(this.velocity, velocityPrev);
                        this.position = plus(this.position, deltaPosition);
                    }

                    if (this.parameters != null && this.parameters.loggingEnabled) {
                        RobotLog.vv(this.parameters.loggingTag, "dt=%.3fs accel=%s vel=%s pos=%s", new Object[]{(double) (this.acceleration.acquisitionTime - accelPrev.acquisitionTime) * 1.0E-9D, this.acceleration, this.velocity, this.position});
                    }
                } else {
                    this.acceleration = linearAcceleration;
                }
            }
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
                int correctionCounter = 0;
                while(!Thread.currentThread().isInterrupted()) {
                    //correct from range sensors every 5 update cycles bc acc = 100hz, range = 20hz
                    correctionCounter += 1;
                    correctionCounter %= 5;
                    if(correctionCounter == 0) RobotNavigation.this.integrator.correctFromRangeSensors(RobotNavigation.this.imu.getAngularOrientation());


                    Acceleration linearAcceleration = RobotNavigation.this.imu.getLinearAcceleration();
                    RobotNavigation.this.integrator.update(linearAcceleration);


                    if (this.msPollInterval > 0) {
                        long msSoFar = (System.nanoTime() - linearAcceleration.acquisitionTime) / 1000000L;
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

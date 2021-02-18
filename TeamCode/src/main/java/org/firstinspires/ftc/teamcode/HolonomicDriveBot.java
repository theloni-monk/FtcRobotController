/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import net.frogbots.ftcopmodetunercommon.opmode.TunableLinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.TimerTask;


@TeleOp(name="POV EMU HOLO Driving OpMode", group="Linear Opmode")
//@Disabled
/**
 * Emulates POV driving for holonomic drivetrain
 */
public class HolonomicDriveBot extends TunableLinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    protected DcMotorImplEx r1 = null;
    protected DcMotorImplEx r2 = null;
    protected DcMotorImplEx l1 = null;
    protected DcMotorImplEx l2 = null;

    protected double leftAngVel = 0;
    protected double rightAngVel = 0;

    public double triggerPower;

    public final double MAX_RPM = 6000; //free speed rpm
    public final double GEAR_RATIO = 20; // 20:1 reduction
    public final double WHEEL_RAD = 37.5; //MM
    public final double     COUNTS_PER_MOTOR_REV    = 28;
    public final double     COUNTS_PER_MM         = (COUNTS_PER_MOTOR_REV * GEAR_RATIO) /
            (WHEEL_RAD * 2 * 3.1415);


    private final double MAX_RAD_PER_SEC = MAX_RPM / 60 / GEAR_RATIO * 2 * Math.PI;
    private final double MAX_VEL_LIN  = MAX_RAD_PER_SEC * (WHEEL_RAD*WHEEL_RAD) ; //meters/sec  V = w * r^2 get linear vel by taking angular vel (rpm in rads) * r^2


    protected Boolean debounced = Boolean.TRUE;

    TimerTask debounce = new TimerTask() {
        @Override
        public void run() {
            synchronized (debounced){
                debounced = false;
                sleep(500);
                debounced = true;
            }
        }
    };

    public void initDriveOp(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        r1  = (DcMotorImplEx) hardwareMap.get(DcMotor.class, "right_drive1");
        r2 = (DcMotorImplEx) hardwareMap.get(DcMotor.class, "right_drive2");
        l1  = (DcMotorImplEx) hardwareMap.get(DcMotor.class, "left_drive1");
        l2 = (DcMotorImplEx) hardwareMap.get(DcMotor.class, "left_drive2");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        r1.setDirection(DcMotor.Direction.FORWARD);
        r2.setDirection(DcMotor.Direction.FORWARD);
        l1.setDirection(DcMotor.Direction.REVERSE);
        l2.setDirection(DcMotor.Direction.REVERSE);

        runtime.reset();
    }

    public void runLeftMotorVel(double angVel, AngleUnit unit){
        l1.setVelocity(angVel, unit);
        l2.setVelocity(angVel, unit);
    }

    public void runRightMotorVel(double angVel, AngleUnit unit){
        r1.setVelocity(angVel, unit);
        r2.setVelocity(angVel, unit);
    }

    public void runLeftMotorPow(double power){
        l1.setPower(power);
        l2.setPower(power);
    }
    public void runRightMotorPow(double power){
        r1.setPower(power);
        r2.setPower(power);
    }

    public void executeControllerDriveLogic(){
        triggerPower = gamepad1.right_trigger * MAX_RAD_PER_SEC; // master speed on trigger
        if (gamepad1.b) triggerPower = 0; // press B to stop

        // POV Mode uses left stick to go forward, and right stick to turn.
        double turn = gamepad1.right_stick_x;
        double drive = gamepad1.left_stick_y;


        this.leftAngVel = Range.clip(triggerPower * (drive + turn), -MAX_RAD_PER_SEC, MAX_RAD_PER_SEC);
        this.rightAngVel = Range.clip(triggerPower * (drive - turn), -MAX_RAD_PER_SEC, MAX_RAD_PER_SEC);

        // Send calculated power to wheels
        runLeftMotorVel(this.leftAngVel, AngleUnit.RADIANS);
        runRightMotorVel(this.rightAngVel, AngleUnit.RADIANS);


        telemetry.addData("Instructions", "Left stick y for direction, right stick x for turning, right trigger for master scaling");
        telemetry.addData("Status", "Run Time: " + runtime.toString() + " MotorController: " + r1.getController().toString());
        // Shift speeds to linear speed when reporting
        telemetry.addData("Motors", "Speeds: left (%.2f)m/s, right (%.2f)m/s", 2 * this.leftAngVel * (WHEEL_RAD * WHEEL_RAD), 2 * this.rightAngVel * (WHEEL_RAD * WHEEL_RAD));
        telemetry.update();
    }


    @Override
    public void runOpMode() {
        initDriveOp();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            executeControllerDriveLogic();
        }
    }
}

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.botfunctionality.AutoBot;
@TeleOp (name="Move and Shoot", group="Performing")
public class MoveNShoot extends AutoBot {


    private DcMotor lShooter = null;
    private DcMotor rShooter = null;
    private ServoImplEx lServo = null;

    @Override
    public void runOpMode() throws InterruptedException {
        lShooter =  hardwareMap.get(DcMotor.class, "shoot_left");
        rShooter = hardwareMap.get(DcMotor.class, "shoot_right");
        lServo = (ServoImplEx) hardwareMap.get(Servo.class, "shoot_servo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        lShooter.setDirection(DcMotor.Direction.FORWARD);
        rShooter.setDirection(DcMotor.Direction.REVERSE);

        lServo.setDirection(Servo.Direction.FORWARD);
        initDriveOp();
        waitForStart();

        boolean spinMotors = false;
        while(opModeIsActive()) {
            executeControllerDriveLogic();
            if(gamepad1.a && debounced) {spinMotors = !spinMotors; debounce.run();};
            if(gamepad1.y && debounced) {lServo.setPosition(1); debounce.run();}
            if(gamepad1.x && debounced) {lServo.setPosition(0.5); debounce.run();}

             // Send calculated power to wheels
            if(spinMotors) {
                lShooter.setPower(-1);
                rShooter.setPower(-1);
            }
            else{
                lShooter.setPower(0);
                rShooter.setPower(0);
            }
        }
    }
}

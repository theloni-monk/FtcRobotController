package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.botfunctionality.AutoBot;

@TeleOp(name = "Move and Shoot", group = "Performing")
public class MoveNShoot extends AutoBot {


    private DcMotor lShooter = null;
    private DcMotor rShooter = null;
    private ServoImplEx lServo = null;

    final double SHOOT_DIST_MM = 500; //FIXME: figure out dist

    @Override
    public void runOpMode() throws InterruptedException {
        lShooter = hardwareMap.get(DcMotor.class, "shoot_left");
        rShooter = hardwareMap.get(DcMotor.class, "shoot_right");
        lServo = (ServoImplEx) hardwareMap.get(Servo.class, "shoot_servo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        lShooter.setDirection(DcMotor.Direction.FORWARD);
        rShooter.setDirection(DcMotor.Direction.REVERSE);

        lServo.setDirection(Servo.Direction.FORWARD);
        initDriveOp();
        waitForStart();

        boolean spinShooter = false;
        boolean feedShooter = false;
        while (opModeIsActive()) {
            //pov  via sticks and trigger
            executeControllerDriveLogic();

            //start/stop shooter from spinning
            if (gamepad1.a && debounced) {
                spinShooter = !spinShooter;
                if (spinShooter) {
                    lShooter.setPower(-1);
                    rShooter.setPower(-1);
                } else {
                    lShooter.setPower(0);
                    rShooter.setPower(0);
                }
                debounce.run();
            }
            ;

            //start/stop servo feed
            if (gamepad1.x && debounced) {
                feedShooter = !feedShooter;
                if (feedShooter) {
                    lServo.setPosition(1);
                } else {
                    lServo.setPosition(0.5);
                }
                debounce.run();
            }

            //straighten robot
            if (gamepad1.y && debounced) {
                straightenRobot();
                debounce.run();
            }

            if(gamepad1.b && debounced){ //WRITEME isntructions
                straightenRobot();
                double yPos = this.rNav.getPosition().toUnit(DistanceUnit.MM).y;
                driveDist(yPos - SHOOT_DIST_MM, DistanceUnit.MM);
                debounce.run();
            }

        }
    }

    void straightenRobot() {
        double ang = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle; //xy plane
        if (ang < 0) ang = 360 - ang;//normalize angle
        rotate((int) (180 - ang), 1); // set robot to 180 deg => shooter facing forward
    }

}

package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.botfunctionality.AutoBot;

@Autonomous(name = "Shoot disks", group = "Linear Opmode")
public class ShootDisksOp extends AutoBot {
    private DcMotor lShooter = null;
    private DcMotor rShooter = null;
    private ServoImplEx rServo = null;

    @Override
    public void runOpMode(){
        boolean runOnce = true;
        initAutoBot();
        composeTelemetry();
        waitForStart();
        while(opModeIsActive() && runOnce){
            executeDeliveryScript();
            executeShootScript();
            runOnce = false;
        }
    }

    public void initShooter(){
        lShooter =  hardwareMap.get(DcMotor.class, "shoot_left");
        rShooter = hardwareMap.get(DcMotor.class, "shoot_right");
        rServo = (ServoImplEx) hardwareMap.get(Servo.class, "shoot_servo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        lShooter.setDirection(DcMotor.Direction.FORWARD);
        rShooter.setDirection(DcMotor.Direction.REVERSE);

        rServo.setDirection(Servo.Direction.FORWARD);
    }

    public void executeShootScript(){

        driveDist(1.5, DistanceUnit.METER);
        telemetry.update();

        double powerLevel = 1;
        lShooter.setPower(powerLevel);
        rShooter.setPower(-powerLevel);
        rServo.setPosition(powerLevel);

        //TODO: test shoot all disks in 25s ?
        try{Thread.sleep(250000);}catch(InterruptedException e){};

        telemetry.update();
    }


    //TODO: make this a lamda so it can be run with other scripts
    public void executeDeliveryScript(){
        driveDist(1.4, DistanceUnit.METER);
        telemetry.update();
        sleep(5000);

        rotate(90, 0.7);
        telemetry.update();
        sleep(5000);

        rotate(-90,0.7);
        telemetry.update();
        sleep(5000);

        driveDist(-1.4, DistanceUnit.METER);
        telemetry.update();
    }
}

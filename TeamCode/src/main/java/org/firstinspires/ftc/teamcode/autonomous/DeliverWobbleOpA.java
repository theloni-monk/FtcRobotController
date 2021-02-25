package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.botfunctionality.PlaybackBot;

@Autonomous(name = "Wobble Delivery A", group = "Auto")
public class DeliverWobbleOpA extends PlaybackBot {


    String jsonstr = "{\"25\":\"-31.41592653589793,-31.41592653589793\",\"34\":\"-31.41592653589793,-31.41592653589793\",\"41\":\"-31.41592653589793,-31.41592653589793\",\"48\":\"-31.41592653589793,-31.41592653589793\",\"55\":\"-31.41592653589793,-31.41592653589793\",\"62\":\"-31.41592653589793,-31.41592653589793\",\"69\":\"-31.41592653589793,-31.41592653589793\",\"77\":\"-31.41592653589793,-31.41592653589793\",\"83\":\"-31.41592653589793,-31.41592653589793\",\"90\":\"-31.41592653589793,-31.41592653589793\",\"98\":\"-31.41592653589793,-31.41592653589793\",\"106\":\"-31.41592653589793,-31.41592653589793\",\"112\":\"-31.41592653589793,-31.41592653589793\",\"118\":\"-31.41592653589793,-31.41592653589793\",\"125\":\"-31.41592653589793,-31.41592653589793\",\"132\":\"-31.41592653589793,-31.41592653589793\",\"138\":\"-31.41592653589793,-31.41592653589793\",\"146\":\"-31.41592653589793,-31.41592653589793\",\"152\":\"-25.74874199857155,-25.74874199857155\",\"159\":\"-25.74874199857155,-25.74874199857155\",\"165\":\"-25.74874199857155,-25.74874199857155\",\"171\":\"-25.74874199857155,-25.74874199857155\",\"177\":\"-25.74874199857155,-25.74874199857155\",\"184\":\"-25.74874199857155,-25.74874199857155\",\"191\":\"-22.052748989170198,-22.052748989170198\",\"197\":\"-22.052748989170198,-22.052748989170198\",\"203\":\"-22.052748989170198,-22.052748989170198\",\"210\":\"-22.052748989170198,-22.052748989170198\",\"217\":\"-22.052748989170198,-22.052748989170198\",\"223\":\"-22.052748989170198,-22.052748989170198\",\"229\":\"-21.806349580045783,-21.806349580045783\",\"235\":\"-21.806349580045783,-21.806349580045783\",\"241\":\"-21.806349580045783,-21.806349580045783\",\"247\":\"-21.806349580045783,-21.806349580045783\",\"253\":\"-21.806349580045783,-21.806349580045783\",\"260\":\"-21.806349580045783,-21.806349580045783\",\"266\":\"-21.806349580045783,-21.806349580045783\",\"272\":\"-19.83515430705047,-19.83515430705047\",\"278\":\"-19.83515430705047,-19.83515430705047\",\"285\":\"-19.83515430705047,-19.83515430705047\",\"291\":\"-19.83515430705047,-19.83515430705047\",\"297\":\"-19.83515430705047,-19.83515430705047\",\"305\":\"-19.83515430705047,-19.83515430705047\",\"311\":\"-12.566371737880257,-12.566371737880257\",\"317\":\"-12.566371737880257,-12.566371737880257\",\"324\":\"-12.566371737880257,-12.566371737880257\",\"331\":\"-12.566371737880257,-12.566371737880257\",\"339\":\"-12.566371737880257,-12.566371737880257\",\"347\":\"-1.093789477782469,-1.093789477782469\",\"354\":\"-1.093789477782469,-1.093789477782469\",\"360\":\"-1.093789477782469,-1.093789477782469\",\"367\":\"-1.093789477782469,-1.093789477782469\",\"373\":\"-1.093789477782469,-1.093789477782469\",\"379\":\"-1.093789477782469,-1.093789477782469\",\"386\":\"-1.093789477782469,-1.093789477782469\",\"22784\":\"1.5726081962793346,1.5726081962793346\",\"22791\":\"1.5726081962793346,1.5726081962793346\",\"22798\":\"1.5726081962793346,1.5726081962793346\",\"22804\":\"1.5726081962793346,1.5726081962793346\",\"22810\":\"1.5726081962793346,1.5726081962793346\",\"22815\":\"1.5726081962793346,1.5726081962793346\",\"22823\":\"1.5726081962793346,1.5726081962793346\",\"22829\":\"14.783966419999985,14.783966419999985\",\"22835\":\"14.783966419999985,14.783966419999985\",\"22840\":\"14.783966419999985,14.783966419999985\",\"22846\":\"14.783966419999985,14.783966419999985\",\"22852\":\"14.783966419999985,14.783966419999985\",\"22858\":\"14.783966419999985,14.783966419999985\",\"22864\":\"14.783966419999985,14.783966419999985\",\"22870\":\"30.430330771935417,30.430330771935417\",\"22876\":\"30.430330771935417,30.430330771935417\",\"22881\":\"30.430330771935417,30.430330771935417\",\"22887\":\"30.430330771935417,30.430330771935417\",\"22893\":\"30.430330771935417,30.430330771935417\",\"22900\":\"30.430330771935417,30.430330771935417\",\"22907\":\"31.41592653589793,31.41592653589793\",\"22913\":\"31.41592653589793,31.41592653589793\",\"22919\":\"31.41592653589793,31.41592653589793\",\"22925\":\"31.41592653589793,31.41592653589793\",\"22931\":\"31.41592653589793,31.41592653589793\",\"22938\":\"31.41592653589793,31.41592653589793\",\"22943\":\"31.41592653589793,31.41592653589793\",\"22950\":\"31.41592653589793,31.41592653589793\",\"22956\":\"31.41592653589793,31.41592653589793\",\"22962\":\"31.41592653589793,31.41592653589793\",\"22968\":\"31.41592653589793,31.41592653589793\",\"22976\":\"31.41592653589793,31.4159265\" }";

    @Override
    public void runOpMode() {
        boolean runOnce = true;
        initPlaybackBot(jsonstr);
        waitForStart();
        while (opModeIsActive() && runOnce) {
            executeDeliveryScript();
            runOnce = false;
        }
    }

    //TODO: make this a lamda so it can be run with other scripts
    public void executeDeliveryScript() {
        executePlaybackLogic();

    }
}

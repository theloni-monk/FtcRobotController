package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.botfunctionality.PlaybackBot;

@Autonomous(name = "Wobble Delivery B", group = "Auto")
public class DeliverWobbleOpB extends PlaybackBot {
    final String fileName = "{\n" +
            "   \"186\": \"-0.6288993684841213,-0.6288993684841213\",\n" +
            "   \"226\": \"-2.254703850802664,-2.254703850802664\",\n" +
            "   \"262\": \"-5.017369122352702,-5.017369122352702\",\n" +
            "   \"307\": \"-8.12485953256463,-8.12485953256463\",\n" +
            "   \"346\": \"-8.214291042358427,-8.214291042358427\",\n" +
            "   \"386\": \"-6.646874186829652,-6.646874186829652\",\n" +
            "   \"426\": \"-4.806793833498366,-4.806793833498366\",\n" +
            "   \"504\": \"-5.243775040772775,-5.243775040772775\",\n" +
            "   \"546\": \"-4.6671469850622795,-4.6671469850622795\",\n" +
            "   \"585\": \"-4.600330337838596,-4.600330337838596\",\n" +
            "   \"624\": \"-4.533513213302005,-4.533513213302005\",\n" +
            "   \"703\": \"-4.463766864328802,-4.463766864328802\",\n" +
            "   \"746\": \"-4.95199130714122,-4.95199130714122\",\n" +
            "   \"790\": \"-5.068046868404386,-5.068046868404386\",\n" +
            "   \"825\": \"-7.017295436044609,-7.017295436044609\",\n" +
            "   \"863\": \"-7.583798737111625,-7.583798737111625\",\n" +
            "   \"903\": \"-6.882223419734158,-6.882223419734158\",\n" +
            "   \"945\": \"-6.642145879795658,-6.642145879795658\",\n" +
            "   \"987\": \"-6.161990799918659,-6.161990799918659\",\n" +
            "   \"1029\": \"-5.121654793518494,-5.121654793518494\",\n" +
            "   \"1184\": \"-5.041628946872327,-5.041628946872327\",\n" +
            "   \"1225\": \"-4.801551406933829,-4.801551406933829\",\n" +
            "   \"1263\": \"-4.561473866995328,-4.561473866995328\",\n" +
            "   \"1305\": \"-4.481448020349162,-4.481448020349162\",\n" +
            "   \"1387\": \"-3.921267093825996,-3.921267093825996\",\n" +
            "   \"1426\": \"-3.6011634031594117,-3.6011634031594117\",\n" +
            "   \"1469\": \"-3.5211375565132452,-3.5211375565132452\",\n" +
            "   \"1507\": \"-3.361085863220912,-3.361085863220912\",\n" +
            "   \"1550\": \"-3.2810600165747457,-3.2810600165747457\",\n" +
            "   \"1744\": \"-4.241370480410662,-4.241370480410662\",\n" +
            "   \"1785\": \"-4.561473866995328,-4.561473866995328\",\n" +
            "   \"1825\": \"-4.721525560287661,-4.721525560287661\",\n" +
            "   \"1867\": \"-4.961603100226161,-4.961603100226161\",\n" +
            "   \"1905\": \"-5.041628946872327,-5.041628946872327\",\n" +
            "   \"1944\": \"-5.121654793518494,-5.121654793518494\",\n" +
            "   \"2036\": \"-5.2016806401646605,-5.2016806401646605\",\n" +
            "   \"2112\": \"-4.721525560287661,-4.721525560287661\",\n" +
            "   \"2266\": \"-5.281706486810827,-5.281706486810827\",\n" +
            "   \"2302\": \"-5.673766262112385,-5.673766262112385\",\n" +
            "   \"2346\": \"-5.827804394072726,-5.827804394072726\",\n" +
            "   \"2423\": \"-6.1319229932587,-6.1319229932587\",\n" +
            "   \"2586\": \"-6.205935278572874,-6.205935278572874\",\n" +
            "   \"2666\": \"-5.6025805235067665,-5.6025805235067665\",\n" +
            "   \"2709\": \"-5.516386987068751,-5.516386987068751\",\n" +
            "   \"2783\": \"-5.34399991419272,-5.34399991419272\",\n" +
            "   \"2946\": \"-4.913032232002643,-4.913032232002643\",\n" +
            "   \"2987\": \"-4.8268386955646285,-4.8268386955646285\",\n" +
            "   \"3224\": \"-4.568258086250582,-4.568258086250582\",\n" +
            "   \"3274\": \"-2.585806420658345,-2.585806420658345\",\n" +
            "   \"3305\": \"-2.0686452020302526,-2.0686452020302526\",\n" +
            "   \"3347\": \"-1.046657982551821,-1.046657982551821\",\n" +
            "   \"3386\": \"-0.6977719607492242,-0.6977719607492242\",\n" +
            "   \"3426\": \"-0.6105504760125674,-0.6105504760125674\",\n" +
            "   \"3551\": \"-0.5233289912759105,-0.5233289912759105\",\n" +
            "   \"3582\": \"-0.3488859803746121,-0.3488859803746121\",\n" +
            "   \"3622\": \"-0.28895655207144827,-0.2343724392044622\",\n" +
            "   \"3664\": \"-0.35063351207576227,-0.17269547920014822\",\n" +
            "   \"3703\": \"-0.12818527453925782,-0.04625771564804822\",\n" +
            "   \"3745\": \"-0.0,-0.0\",\n" +
            "   \"3989\": \"-0.4554843487644344,-0.4554843487644344\",\n" +
            "   \"4027\": \"-0.6073124409810864,-0.6073124409810864\",\n" +
            "   \"4064\": \"-0.0,-0.0\",\n" +
            "   \"5552\": \"-1.1280716255882854,1.1280716255882854\",\n" +
            "   \"5585\": \"-3.4566338299903623,3.4566338299903623\",\n" +
            "   \"5622\": \"-5.946687135362905,5.946687135362905\",\n" +
            "   \"5664\": \"-5.313367261286121,5.313367261286121\",\n" +
            "   \"5707\": \"-5.70609505256183,5.70609505256183\",\n" +
            "   \"5747\": \"-6.872097775382581,6.872097775382581\",\n" +
            "   \"5784\": \"-4.671875096757333,4.671875096757333\",\n" +
            "   \"5828\": \"-2.5764007536047204,2.5764007536047204\",\n" +
            "   \"5862\": \"-0.8690797255166729,0.8690797255166729\",\n" +
            "   \"5903\": \"0.0,0.0\",\n" +
            "   \"6266\": \"-0.0348988816017269,-0.0348988816017269\",\n" +
            "   \"6303\": \"-1.9203121679330444,-1.9203121679330444\",\n" +
            "   \"6343\": \"-3.2708832753747465,-3.2708832753747465\",\n" +
            "   \"6386\": \"-3.3263924913133747,-3.3263924913133747\",\n" +
            "   \"6465\": \"-5.17438805974648,-5.17438805974648\",\n" +
            "   \"6506\": \"-8.007981732811027,-8.007981732811027\",\n" +
            "   \"6548\": \"-8.131181437373234,-8.131181437373234\",\n" +
            "   \"6703\": \"-7.02238409631337,-7.02238409631337\",\n" +
            "   \"6749\": \"-6.652784982626749,-6.652784982626749\",\n" +
            "   \"6785\": \"-6.159986164377922,-6.159986164377922\",\n" +
            "   \"6826\": \"-5.6671868779953085,-5.6671868779953085\",\n" +
            "   \"6866\": \"-5.543987173433101,-5.543987173433101\",\n" +
            "   \"6983\": \"-5.17438805974648,-5.17438805974648\",\n" +
            "   \"7025\": \"-5.051188355184273,-5.051188355184273\",\n" +
            "   \"7343\": \"-4.9279886506220665,-4.9279886506220665\",\n" +
            "   \"8105\": \"-4.804788946059859,-4.804788946059859\",\n" +
            "   \"9549\": \"-4.5583895369354455,-4.5583895369354455\",\n" +
            "   \"9597\": \"-4.311990127811031,-4.311990127811031\",\n" +
            "   \"9948\": \"-4.065590718686617,-4.065590718686617\",\n" +
            "   \"9982\": \"-3.572791900437789,-3.572791900437789\",\n" +
            "   \"10027\": \"-3.449592195875582,-3.449592195875582\",\n" +
            "   \"10424\": \"-3.3263924913133747,-3.3263924913133747\",\n" +
            "   \"10465\": \"-3.203192786751168,-3.203192786751168\",\n" +
            "   \"10665\": \"-3.4664506231637526,-2.939934950338583\",\n" +
            "   \"10705\": \"-3.7871707778682837,-2.619214795634052\",\n" +
            "   \"10749\": \"-4.0009842938901246,-2.4054012796122115\",\n" +
            "   \"10865\": \"-4.054437613231469,-2.3519479602708664\",\n" +
            "   \"10905\": \"-4.268251081522018,-2.138134491980318\",\n" +
            "   \"10946\": \"-4.181183828285434,-1.9788023360924882\",\n" +
            "   \"10986\": \"-3.928205198705245,-1.4925822701656488\",\n" +
            "   \"11024\": \"-4.176968950787534,-1.24381851808336\",\n" +
            "   \"11064\": \"-4.3126582994782465,-1.108129169392647\",\n" +
            "   \"11143\": \"-2.9042650253209286,-1.7773242161767233\",\n" +
            "   \"11189\": \"-2.094395211624412,-2.094395211624412\",\n" +
            "   \"11266\": \"-2.8335934389976543,-2.8335934389976543\",\n" +
            "   \"11743\": \"-2.9567933776267536,-2.9567933776267536\",\n" +
            "   \"11786\": \"-0.5143858506767205,-0.5143858506767205\",\n" +
            "   \"11827\": \"0.39103194719046297,-0.39103194719046297\",\n" +
            "   \"11864\": \"2.710085570654239,-2.710085570654239\",\n" +
            "   \"11902\": \"3.203192786751168,-3.203192786751168\",\n" +
            "   \"11948\": \"3.449592195875582,-3.449592195875582\",\n" +
            "   \"11983\": \"2.7588102286094975,-2.7588102286094975\",\n" +
            "   \"12029\": \"0.9455077444462787,-0.9455077444462787\",\n" +
            "   \"12068\": \"0.0,0.0\",\n" +
            "   \"12222\": \"1.2684894225148005,-1.2684894225148005\",\n" +
            "   \"12266\": \"3.7766855655827376,-3.7766855655827376\",\n" +
            "   \"12308\": \"4.9279886506220665,-4.9279886506220665\",\n" +
            "   \"12347\": \"5.420787468870894,-5.420787468870894\",\n" +
            "   \"12388\": \"6.529585278064543,-6.529585278064543\",\n" +
            "   \"12428\": \"6.652784982626749,-6.652784982626749\",\n" +
            "   \"12504\": \"5.22280503672136,-5.22280503672136\",\n" +
            "   \"12549\": \"1.4235042630049124,-1.4235042630049124\",\n" +
            "   \"12584\": \"0.0,0.0\",\n" +
            "   \"12785\": \"2.561495414803574,-2.561495414803574\",\n" +
            "   \"12824\": \"6.44919950018757,-6.44919950018757\",\n" +
            "   \"12864\": \"7.515182914562199,-7.515182914562199\",\n" +
            "   \"12985\": \"1.3288300667414998,-1.3288300667414998\",\n" +
            "   \"13026\": \"0.0,0.0\",\n" +
            "   \"13346\": \"-0.7972775282633785,-0.7972775282633785\",\n" +
            "   \"13389\": \"-3.502840284791805,-3.502840284791805\",\n" +
            "   \"13426\": \"-4.136056933910261,-4.136056933910261\",\n" +
            "   \"13465\": \"-4.193622046446172,-4.193622046446172\",\n" +
            "   \"13504\": \"-1.1426684090905619,-1.1426684090905619\",\n" +
            "   \"13545\": \"0.5094516057633179,-0.5094516057633179\",\n" +
            "   \"13585\": \"3.1574489927411866,-3.1574489927411866\",\n" +
            "   \"13630\": \"6.726489054143431,-6.726489054143431\",\n" +
            "   \"13666\": \"6.899184391751164,-6.899184391751164\",\n" +
            "   \"13703\": \"6.154075476698831,-6.154075476698831\",\n" +
            "   \"13748\": \"5.532166266208707,-5.532166266208707\",\n" +
            "   \"13787\": \"1.0091891893955816,-1.0091891893955816\",\n" +
            "   \"13824\": \"0.0,0.0\",\n" +
            "   \"14026\": \"-0.5837210223615847,-0.5837210223615847\",\n" +
            "   \"14065\": \"-3.347362843506355,-3.347362843506355\",\n" +
            "   \"14103\": \"-7.45247806053308,-7.45247806053308\",\n" +
            "   \"14145\": \"-7.638382619124406,-7.638382619124406\",\n" +
            "   \"14186\": \"-8.007981732811027,-8.007981732811027\",\n" +
            "   \"14382\": \"-7.07254819436654,-7.07254819436654\",\n" +
            "   \"14422\": \"-2.3285621843252504,-2.3285621843252504\",\n" +
            "   \"14468\": \"0.0,0.0\",\n" +
            "   \"14825\": \"-5.86984758971441,5.86984758971441\",\n" +
            "   \"14866\": \"-7.941165085587342,7.941165085587342\",\n" +
            "   \"14906\": \"-8.007981732811027,8.007981732811027\",\n" +
            "   \"14944\": \"-7.139364841590225,7.139364841590225\",\n" +
            "   \"14984\": \"-2.595829011876441,2.595829011876441\",\n" +
            "   \"15026\": \"0.0,0.0\",\n" +
            "   \"15265\": \"-0.870107685884846,0.870107685884846\",\n" +
            "   \"15312\": \"-4.308752271517994,4.308752271517994\",\n" +
            "   \"15346\": \"-6.899184391751164,6.899184391751164\",\n" +
            "   \"15427\": \"-1.488059495529463,1.488059495529463\",\n" +
            "   \"15461\": \"0.0,0.0\",\n" +
            "   \"15549\": \"-0.0480566621738954,-0.0480566621738954\",\n" +
            "   \"15587\": \"-0.5799690398978069,-0.5799690398978069\",\n" +
            "   \"15626\": \"-0.6716620417685489,-0.6716620417685489\",\n" +
            "   \"15668\": \"-1.131617967926847,-1.131617967926847\",\n" +
            "   \"15706\": \"-2.4609105902712893,-2.4609105902712893\",\n" +
            "   \"15746\": \"-3.94115730898935,-3.94115730898935\",\n" +
            "   \"15784\": \"-5.029396251266276,-5.029396251266276\",\n" +
            "   \"15824\": \"-10.781543484632213,0.0\",\n" +
            "   \"15865\": \"-12.472930933100306,0.832639032153192\",\n" +
            "   \"15907\": \"-13.395412584527516,0.40295619897481194\",\n" +
            "   \"15947\": \"-11.056519997231018,-2.753875952057514\",\n" +
            "   \"15985\": \"-7.085962791742384,-7.085962791742384\",\n" +
            "   \"16025\": \"-5.3886645252911105,-5.3886645252911105\",\n" +
            "   \"16064\": \"-1.6890749808630225,-1.6890749808630225\",\n" +
            "   \"16106\": \"0.0,0.0\",\n" +
            "   \"16344\": \"-1.0642358648474803,-1.0642358648474803\",\n" +
            "   \"16388\": \"-5.6461142499232935,-0.6558317393301097\",\n" +
            "   \"16424\": \"-10.952388534832389,0.9539369979510716\",\n" +
            "   \"16461\": \"-13.9930621301752,0.29810547157595396\",\n" +
            "   \"16504\": \"-14.231546592617962,0.05962100913319395\",\n" +
            "   \"16543\": \"-9.938830314793131,-4.352337286958024\",\n" +
            "   \"16582\": \"-7.145583800875578,-7.145583800875578\",\n" +
            "   \"16703\": \"-3.5683204844690435,-3.5683204844690435\",\n" +
            "   \"16747\": \"0.0,0.0\",\n" +
            "   \"16944\": \"-0.37926185743802027,0.37926185743802027\",\n" +
            "   \"16983\": \"-4.244351181007115,4.244351181007115\",\n" +
            "   \"17025\": \"-5.548818367712862,5.548818367712862\",\n" +
            "   \"17066\": \"-5.790386582557515,5.790386582557515\",\n" +
            "   \"17154\": \"-6.652784982626749,6.652784982626749\",\n" +
            "   \"17186\": \"-2.8226459105663286,2.8226459105663286\",\n" +
            "   \"17230\": \"0.0,0.0\",\n" +
            "   \"17383\": \"-0.041940360560516,-0.041940360560516\",\n" +
            "   \"17424\": \"-0.8314054205277052,-0.8314054205277052\",\n" +
            "   \"17466\": \"-3.7918993685558124,-3.7918993685558124\",\n" +
            "   \"17506\": \"-5.666878772042374,-5.666878772042374\",\n" +
            "   \"17544\": \"-5.913586755253507,-5.913586755253507\",\n" +
            "   \"17827\": \"-5.370829615161698,-5.370829615161698\",\n" +
            "   \"17866\": \"-0.7820638943809259,-0.7820638943809259\",\n" +
            "   \"17905\": \"0.0,0.0\",\n" +
            "   \"18104\": \"-1.4687853422034922,-1.4687853422034922\",\n" +
            "   \"18145\": \"-3.0703302947489846,-3.0703302947489846\",\n" +
            "   \"18186\": \"-4.461145676005859,-4.461145676005859\",\n" +
            "   \"18225\": \"-4.966896586883683,-4.966896586883683\",\n" +
            "   \"18263\": \"-5.051188355184273,-5.051188355184273\",\n" +
            "   \"18346\": \"-4.418999490781277,-4.418999490781277\",\n" +
            "   \"18385\": \"-0.8309428800409759,-0.8309428800409759\",\n" +
            "   \"18427\": \"0.0,0.0\",\n" +
            "   \"18742\": \"-3.1273815783215055,-3.1273815783215055\",\n" +
            "   \"18787\": \"-9.363178482995304,-9.363178482995304\",\n" +
            "   \"18906\": \"-9.486378187557511,-9.486378187557511\",\n" +
            "   \"18956\": \"-4.3414922043361655,-4.3414922043361655\",\n" +
            "   \"18982\": \"0.0,0.0\",\n" +
            "   \"19468\": \"2.3118066044807284,2.3118066044807284\",\n" +
            "   \"19504\": \"9.885274075005055,9.885274075005055\",\n" +
            "   \"19544\": \"11.334373755990617,11.334373755990617\",\n" +
            "   \"19588\": \"11.457573460552824,11.457573460552824\",\n" +
            "   \"19626\": \"11.703972869677237,11.703972869677237\",\n" +
            "   \"19704\": \"11.827173510507015,11.827173510507015\",\n" +
            "   \"19747\": \"11.950373215069222,11.950373215069222\",\n" +
            "   \"19786\": \"12.566371737880257,12.566371737880257\",\n" +
            "   \"19829\": \"12.812771147004671,12.812771147004671\",\n" +
            "   \"19906\": \"13.182370260691293,13.182370260691293\",\n" +
            "   \"19943\": \"14.04476819262674,14.04476819262674\",\n" +
            "   \"19986\": \"14.660766715437777,14.660766715437777\",\n" +
            "   \"20024\": \"15.276765238248812,15.276765238248812\",\n" +
            "   \"20069\": \"15.76956405649764,15.76956405649764\",\n" +
            "   \"20107\": \"16.13916317018426,16.13916317018426\",\n" +
            "   \"20146\": \"16.63196198843309,16.63196198843309\",\n" +
            "   \"20187\": \"16.8783613975575,16.8783613975575\",\n" +
            "   \"20224\": \"17.124760806681916,17.124760806681916\",\n" +
            "   \"20266\": \"17.494359920368538,17.494359920368538\",\n" +
            "   \"20306\": \"17.617559624930745,17.617559624930745\",\n" +
            "   \"20502\": \"17.740759329492953,17.740759329492953\",\n" +
            "   \"20709\": \"17.86395903405516,17.86395903405516\",\n" +
            "   \"20866\": \"16.37343278001032,16.37343278001032\",\n" +
            "   \"20904\": \"0.0,0.0\",\n" +
            "   \"21069\": \"0.5501071025279385,-0.5501071025279385\",\n" +
            "   \"21105\": \"9.55185873822769,-9.55185873822769\",\n" +
            "   \"21148\": \"16.26745081115578,-16.26745081115578\",\n" +
            "   \"21195\": \"17.124760806681916,-17.124760806681916\",\n" +
            "   \"21262\": \"17.86395903405516,-17.86395903405516\",\n" +
            "   \"21307\": \"18.110358443179575,-18.110358443179575\",\n" +
            "   \"21426\": \"18.233558147741782,-18.233558147741782\",\n" +
            "   \"21461\": \"18.6031572614284,-18.6031572614284\",\n" +
            "   \"21506\": \"14.722648716953671,-14.722648716953671\",\n" +
            "   \"21546\": \"0.0,0.0\",\n" +
            "   \"21789\": \"-1.528920441923795,-1.528920441923795\",\n" +
            "   \"21827\": \"-11.30780190747004,-11.30780190747004\",\n" +
            "   \"21864\": \"-17.050954065405424,-17.050954065405424\",\n" +
            "   \"21906\": \"-18.6031572614284,-18.6031572614284\",\n" +
            "   \"22667\": \"-18.726356965990608,-18.726356965990608\",\n" +
            "   \"22705\": \"-13.88266074991513,-13.88266074991513\",\n" +
            "   \"22745\": \"-2.862427402386945,-2.862427402386945\",\n" +
            "   \"22786\": \"0.0,0.0\",\n" +
            "   \"23145\": \"-4.1676662078132285,4.1676662078132285\",\n" +
            "   \"23183\": \"-6.030310375691173,6.030310375691173\",\n" +
            "   \"23226\": \"-7.892953989151828,7.892953989151828\",\n" +
            "   \"23265\": \"-13.791327464640633,13.791327464640633\",\n" +
            "   \"23303\": \"-17.320738877999535,17.320738877999535\",\n" +
            "   \"23347\": \"-17.987158738617367,17.987158738617367\",\n" +
            "   \"23426\": \"-17.86395903405516,17.86395903405516\",\n" +
            "   \"23467\": \"-17.617559624930745,17.617559624930745\",\n" +
            "   \"23502\": \"-9.091749229277395,9.091749229277395\",\n" +
            "   \"23541\": \"0.0,0.0\",\n" +
            "   \"23866\": \"-3.54678481852058,3.54678481852058\",\n" +
            "   \"23905\": \"-13.170445520930695,13.170445520930695\",\n" +
            "   \"23945\": \"-18.6031572614284,18.6031572614284\",\n" +
            "   \"23986\": \"-12.549564686055335,12.549564686055335\",\n" +
            "   \"24025\": \"-0.13193737665472727,0.13193737665472727\",\n" +
            "   \"24064\": \"0.0,0.0\",\n" +
            "   \"25822\": \"0,0\"\n" +
            "}";
    @Override
    public void runOpMode(){
        boolean runOnce = true;
        initPlaybackBot(fileName);
        waitForStart();
        while(opModeIsActive() && runOnce){
            executeDeliveryScript();
            runOnce = false;
        }
    }

    //TODO: make this a lamda so it can be run with other scripts
    public void executeDeliveryScript(){
        executePlaybackLogic();
    }
}

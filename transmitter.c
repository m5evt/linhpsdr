/* Copyright (C)
* 2018 - John Melton, G0ORX/N6LYT
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
*
*/

#include <gtk/gtk.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <wdsp.h>

#include "ringbuffer.h"
#include "peak_detect.h"

#include "alex.h"
#include "discovered.h"
#include "bpsk.h"
#include "mode.h"
#include "filter.h"
#include "receiver.h"
#include "tx_info_meter.h"
#include "transmitter.h"
#include "wideband.h"
#include "adc.h"
#include "dac.h"
#include "radio.h"
#include "protocol1.h"
#include "protocol2.h"
#include "tx_panadapter.h"
#include "main.h"
#include "vox.h"
#include "audio.h"
#include "mic_level.h"
#include "property.h"
#include "ext.h"
#include "tx_info.h"
#ifdef SOAPYSDR
#include "soapy_protocol.h"
#endif
#ifdef CWDAEMON
#include "cwdaemon.h"
#endif

double ctcss_frequencies[CTCSS_FREQUENCIES]= {
  67.0,71.9,74.4,77.0,79.7,82.5,85.4,88.5,91.5,94.8,
  97.4,100.0,103.5,107.2,110.9,114.8,118.8,123.0,127.3,131.8,
  136.5,141.3,146.2,151.4,156.7,162.2,167.9,173.8,179.9,186.2,
  192.8,203.5,210.7,218.1,225.7,233.6,241.8,250.3
};

#define CW_WAVEFORM_SAMPLES 400
const double cw_waveform[CW_WAVEFORM_SAMPLES] = {
0.0,
0.00000000000000000015403328817757226,
0.00001987264342941043150452505583470,
0.00005984787171592420785192975585254,
0.00012015392732683791740359857680431,
0.00020101730418647359346223657539099,
0.00030266269826417552171043956477092,
0.00042531295834411073000097536223052,
0.00056918903698809737365382765972299,
0.00073450994170266392484236472881776,
0.00092149268632152987398842292066092,
0.00113035224261467015689774928688394,
0.00136130149213510662914439208748263,
0.00161455117831453640736549814960199,
0.00189030985881889204355821298975115,
0.00218878385817487331257980009979747,
0.00251017722067848171943471591305297,
0.00285469166359651662506768943217139,
0.00322252653067199796091402141939852,
0.00361387874594439181472060873545615,
0.00402894276789549608819296366846174,
0.00446791054393177924225488695242348,
0.00493097146521393955853618251694570,
0.00541831232184435650223175429118783,
0.00593011725842308712319805863444344,
0.00646656772998298844112863292821203,
0.00702784245831445959079575303007914,
0.00761411738869029793702614128392270,
0.00822556564700100203979538093790325,
0.00886235749731087603386914253178475,
0.00952466029984515641371167760098615,
0.01021263846941832670323080378693703,
0.01092645343431371957043651832464093,
0.01166626359562440706785224620034569,
0.01243222428706528166764133658261926,
0.01322448773526621948470882728088327,
0.01404320302055600891422137976860540,
0.01488851603824676433918128282130056,
0.01576056946042833986632025755625364,
0.01665950269828225110968489275364846,
0.01758545186492447426340746119421965,
0.01853854973878633211059607788229187,
0.01951892572754276855562238779384643,
0.02052670583259690387234464026278147,
0.02156201261412995714983331652092602,
0.02262496515672534486141032061823353,
0.02371567903557566039940418534115452,
0.02483426628328118043742556153574697,
0.02598083535724837050961966156137350,
0.02715549110769682750188280806469265,
0.02835833474628285041929665055704390,
0.02958946381534782038569275641748391,
0.03084897215779937654223097354133643,
0.03213694988763329818404201887460658,
0.03345348336110380571550848571860115,
0.03479865514854994096305418338488380,
0.03617254400688548615638850947107130,
0.03757522485275973517238057297618070,
0.03900676873639650349412377750013547,
0.04046724281611813189840631821425632,
0.04195671033356173501571717565639119,
0.04347523058959427977310241431041504,
0.04502285892093322444695857598162547,
0.04659964667747912986373037824705534,
0.04820564120036670285873725561032188,
0.04984088580074035046418856609307113,
0.05150541973926030248076557427339139,
0.05319927820634519355369462800808833,
0.05492249230315706526317498514799809,
0.05667508902333369402715135265680146,
0.05845709123547456614877759761839116,
0.06026851766638498253403355420232401,
0.06210938288508378868346682111223345,
0.06397969728757944840591420643249876,
0.06587946708241947113560144089206005,
0.06780869427701739782232692732577561,
0.06976737666476220955935616530041443,
0.07175550781291394064620448034474975,
0.07377307705129017290079929125568015,
0.07582006946174656147885428936206154,
0.07789646586845570819246376004230115,
0.08000224282898808769726173295566696,
0.08213737262619744128322452070278814,
0.08430182326091514855015418561379192,
0.08649555844545557536928725994584966,
0.08871853759793578431125382621758035,
0.09097071583741216105334359554035473,
0.09325204397983635762336973584751831,
0.09556246853483298109299681755146594,
0.09790193170330116489985528005490778,
0.10027037137584188242200866625353228,
0.10266772113201279303940083309498732,
0.10509391024041232765018349937236053,
0.10754886365959415162052437153761275,
0.11003250203981362886906936182640493,
0.11254474172560673117526874875693466,
0.11508549475920289151265052396411193,
0.11765466888477198181828242695701192,
0.12025216755350592867657155693450477,
0.12287788992953561917342852893852978,
0.12553173089668273609831317116913851,
0.12821358106604666127203984160587424,
0.13092332678442614168901059201743919,
0.13366085014357517724015167459583608,
0.13642602899029263041619230989454081,
0.13921873693734460042392697687319014,
0.14203884337521818781446825141756563,
0.14488621348470651084561211519030621,
0.14776070825032208699845170940534445,
0.15066218447453808004787845220562303,
0.15359049479285505346304319118644344,
0.15654548768969200889245030339225195,
0.15952700751509812926443032665702049,
0.16253489450228450485802511593647068,
0.16556898478597226187503110850229859,
0.16862911042155467877812213828292442,
0.17171509940507034830403654268593527,
0.17482677569398435979408645835064817,
0.17796395922877425443964227724791272,
0.18112646595531708970661100011056988,
0.18431410784807406022522968669363763,
0.18752669293406903916476835547655355,
0.19076402531765709880140491350175580,
0.19402590520607765345317829996929504,
0.19731212893579008760269744016113691,
0.20062248899958509684715579624025850,
0.20395677407446824447312394568143645,
0.20731476905031057111905568035581382,
0.21069625505926103947729188803350553,
0.21410100950591579027637578747089719,
0.21752880609823826985049777249514591,
0.22097941487922553860379082379949978,
0.22445260225931371045327011870540446,
0.22794813104951813786946956952306209,
0.23146576049530045993130045189900557,
0.23500524631115776719170185060647782,
0.23856634071592675017114970614784397,
0.24214879246879578156281809242500458,
0.24575234690601904796736221214814577,
0.24937674597832540368536058394965949,
0.25302172828901436929527335450984538,
0.25668702913273266918992021601297893,
0.26037238053492389733278855601383839,
0.26407751129194262373900414786476176,
0.26780214701182725178796317777596414,
0.27154601015572138456022344144003000,
0.27530882007993723314953626868373249,
0.27909029307865301783309064376226161,
0.28289014242723420355929420111351646,
0.28670807842617229699300196443800814,
0.29054380844563149066672735898464452,
0.29439703697059405040903357075876556,
0.29826746564659617488857179523620289,
0.30215479332604544548956937433104031,
0.30605871611510959695579003891907632,
0.30997892742116889275294511207903270,
0.31391511800082039229664587765000761,
0.31786697600842800381926167574420106,
0.32183418704520455611017837327381130,
0.32581643420881872819094837723241653,
0.32981339814351651185120317677501589,
0.33382475709074627134853585630480666,
0.33785018694027835195470288454089314,
0.34188936128180752449523538416542578,
0.34594195145702905103135549325088505,
0.35000762661217749149855649193341378,
0.35408605375101781520541521786071826,
0.35817689778827643820591219991911203,
0.36227982160350441498408713414391968,
0.36639448609535929524128050616127439,
0.37052055023629532071183234620548319,
0.37465767112765063773238694011524785,
0.37880550405512147804643063864205033,
0.38296370254461009619006972570787184,
0.38713191841843674900758287549251691,
0.39130980185190256115390639024553820,
0.39549700143019361764373797996086068,
0.39969316420561273872635865700431168,
0.40389793575512850098974126922257710,
0.40811096023822923672952356355381198,
0.41233188045506952157381874712882563,
0.41656033790489849222282714436005335,
0.42079597284475717122731452946027275,
0.42503842434843214226347640760650393,
0.42928733036565480674084938073065132,
0.43354232778153284355582286480057519,
0.43780305247620227016014382570574526,
0.44206913938468750391308503822074272,
0.44634022255695715575285476006683894,
0.45061593521816289964476709428709000,
0.45489590982904931637520462572865654,
0.45917977814652255474925368616823107,
0.46346717128436376587075073985033669,
0.46775771977407687440830841296701692,
0.47205105362585658701490842759085353,
0.47634680238966370380282455698761623,
0.48064459521639707473283920080575626,
0.48494406091914765699613099059206434,
0.48924482803452262746901624268502928,
0.49354652488402633858655121912306640,
0.49784877963548690438244648248655722,
0.50215122036451353970676336757605895,
0.50645347511597404999150739968172275,
0.51075517196547770559789114486193284,
0.51505593908085267607077639695489779,
0.51935540478360342486752188051468693,
0.52365319761033679579753652433282696,
0.52794894637414380156315019121393561,
0.53224228022592334763629651206429116,
0.53653282871563656719615664769662544,
0.54082022185347788933995616389438510,
0.54510409017095107220285399307613261,
0.54938406478183754444444275577552617,
0.55365977744304295526944770244881511,
0.55793086061531260710921742429491132,
0.56219694752379789637330986806773581,
0.56645767221846743399993329148855992,
0.57071266963434530428145308178500272,
0.57496157565156813529227974868263118,
0.57920402715524299530613916431320831,
0.58343966209510156328832408689777367,
0.58766811954493070047078617790248245,
0.59188903976177109633738382399315014,
0.59610206424487166554371242455090396,
0.60030683579438748331824626802699640,
0.60450299856980682644547187010175548,
0.60869019814809777191300099730142392,
0.61286808158156347303702204953879118,
0.61703629745539012585453519932343625,
0.62119449594487885502047674890491180,
0.62534232887234975084567167868954130,
0.62947944976370495684392381008365192,
0.63360551390464114884792934390134178,
0.63772017839649608461627394717652351,
0.64182310221172422792790257517481223,
0.64591394624898279541724832597537898,
0.64999237338782289707950212687137537,
0.65405804854297144856900558806955814,
0.65811063871819308612742815967067145,
0.66214981305972198111220450300606899,
0.66617524290925422825182522501563653,
0.67018660185648404326030913580325432,
0.67418356579118177140941270408802666,
0.67816581295479583246788024553097785,
0.68213302399157238475879694306058809,
0.68608488199917994077026150989695452,
0.69002107257883138480281104421010241,
0.69394128388489062508881488611223176,
0.69784520667395477655503555070026778,
0.70173253435340388062257943602162413,
0.70560296302940617163557135427254252,
0.70945619155436867586672633478883654,
0.71329192157382781402930049807764590,
0.71710985757276590746300826140213758,
0.72090970692134703767806058749556541,
0.72469117992006260031701003754278645,
0.72845398984427855992862532730214298,
0.73219785298817252616743189719272777,
0.73592248870805720972754215836175717,
0.73962761946507593613375775021268055,
0.74331297086726721978777732147136703,
0.74697827171098551968242418297450058,
0.75062325402167451304791256916359998,
0.75424765309398078549918409407837316,
0.75785120753120405190372821380151436,
0.76143365928407302778424536882084794,
0.76499475368884206627484445562004112,
0.76853423950469934577967023869859986,
0.77205186895048172335265235233237036,
0.77554739774068603974654934063437395,
0.77902058512077432261833109805593267,
0.78247119390176156361604853373137303,
0.78589899049408407094574613438453525,
0.78930374494073873847810318693518639,
0.79268523094968934561421747275744565,
0.79604322592553156123784674491616897,
0.79937751100041465335266366309951991,
0.80268787106420957933039517229190096,
0.80597409479392212450221677499939688,
0.80923597468234254037611208332236856,
0.81247330706593057225717302571865730,
0.81568589215192577324131661953288130,
0.81887353404468266049320845922920853,
0.82203604077122549576017718209186569,
0.82517322430601547367245984787587076,
0.82828490059492965169596345731406473,
0.83137088957844529346630224608816206,
0.83443101521402784914727135401335545,
0.83746510549771546738639926843461581,
0.84047299248490181522441844208515249,
0.84345451231030799110754969660774805,
0.84640950520714486327022996192681603,
0.84933781552546172566309223839198239,
0.85223929174967782973482144370791502,
0.85511378651529335037650980666512623,
0.85796115662478156238535120792221278,
0.86078126306265523304261932935332879,
0.86357397100970711978362714944523759,
0.86633914985642457295966778474394232,
0.86907667321557358075523325169342570,
0.87178641893395292239432592396042310,
0.87446826910331698634593067254172638,
0.87712211007046392285957381318439730,
0.87974783244649357172306736174505204,
0.88234533111522761572587114642374218,
0.88491450524079673378707866504555568,
0.88745525827439286636888482462381944,
0.88996749796018592704172078811097890,
0.89245113634040540429026577839977108,
0.89490608975958729764954568963730708,
0.89733227886798683226032835591468029,
0.89972962862415772899993271494167857,
0.90209806829669858529996417928487062,
0.90443753146516669971788360271602869,
0.90674795602016344808760095475008711,
0.90902928416258765853541490287170745,
0.91128146240206397976635344093665481,
0.91350444155454435524177370098186657,
0.91569817673908482369427019875729457,
0.91786262737380241993889740115264431,
0.91999775717101184291379922797204927,
0.92210353413154411139629473836976103,
0.92417993053825331362105544030782767,
0.92622692294870978546583728530094959,
0.92824449218708593445370524932513945,
0.93023262333523781819621945032849908,
0.93219130572298258829988526485976763,
0.93412053291758057049776198255131021,
0.93602030271242053771629798575304449,
0.93789061711491616968316975544439629,
0.93973148233361492032145179109647870,
0.94154290876452539915675288284546696,
0.94332491097666637536178768641548231,
0.94507750769684295555350672657368705,
0.94680072179365470930179071729071438,
0.94849458026073962813029538665432483,
0.95015911419925969116917485735029913,
0.95179435879963325550789932094630785,
0.95340035332252093258631475691800006,
0.95497714107906683800308655918342993,
0.95652476941040576186026100913295522,
0.95804328966643836906769138295203447,
0.95953275718388197912389614430139773,
0.96099323126360358671149697329383343,
0.96242477514724034115545237000333145,
0.96382745599311470119374689602409489,
0.96520134485145014924256656740908511,
0.96654651663889623591785493772476912,
0.96786305011236672957153359675430693,
0.96915102784220075182730624874238856,
0.97041053618465222818656457093311474,
0.97164166525371720162240762874716893,
0.97284450889230322800926842319313437,
0.97401916464275173357378889704705216,
0.97516573371671899650436898809857666,
0.97628432096442441245898180568474345,
0.97737503484327470371084700673236512,
0.97843798738587006713629534715437330,
0.97947329416740314123046573513420299,
0.98048107427245723144437761220615357,
0.98146145026121378585060028854059055,
0.98241454813507567145336452085757628,
0.98334049730171790848487489711260423,
0.98423943053957174686985354128410108,
0.98511148396175340913316631485940889,
0.98595679697944416108867926595848985,
0.98677551226473392276261620281729847,
0.98756777571293485884496021753875539,
0.98833373640437582885454048664541915,
0.98907354656568635675739642465487123,
0.98978736153058177738017775482148863,
0.99047533970015499971140116031165235,
0.99113764250268920896758118033176288,
0.99177443435299916796310526478919201,
0.99238588261130977752344506370718591,
0.99297215754168566270720930333482102,
0.99353343227001711390755644970340654,
0.99406988274157692675458974918001331,
0.99458168767815580135760455959825777,
0.99506902853478618187210713585955091,
0.99553208945606830315711022194591351,
0.99597105723210466177164335022098385,
0.99638612125405567887526103731943294,
0.99677747346932810135200497825280763,
0.99714530833640357965208522728062235,
0.99748982277932163320599556755041704,
0.99781121614182510803914283314952627,
0.99810969014118122721868076041573659,
0.99838544882168556160451089453999884,
0.99863869850786490811600515371537767,
0.99886964775738540378569041422451846,
0.99907850731367853214237584325019270,
0.99926549005829734539929631864652038,
0.99943081096301189525377139943884686,
0.99957468704165597372934826125856489,
0.99969733730173593322376746073132381,
0.99979898269581357617141748050926253,
0.99987984607267321734980214387178421,
0.99994015212828413208256961297593080,
1.0
};

const int protocol1_tx_scheduler[20][26] = {
//  Three receivers - 25 Tx queue events per set of 63, 126, 252, 504  received frames
{0, 3, 5, 8, 10, 13, 15, 18, 20, 23, 25, 28, 30, 33, 35, 38, 40, 43, 45, 48, 50, 53, 55, 58, 60, -1}, // 25 frames per 63
{0, 6, 10, 16, 20, 26, 30, 36, 40, 46, 50, 56, 60, 66, 70, 76, 80, 86, 90, 96, 100, 106, 110, 116, 120, -1}, // 25 frames per 126
{0, 12, 20, 32, 40, 52, 60, 72, 80, 92, 100, 112, 120, 132, 140, 152, 160, 172, 180, 192, 200, 212, 220, 232, 240}, // 25 frames per 252
{0, 24, 40, 64, 80, 104, 120, 144, 160, 184, 200, 224,240, 264, 280, 304, 320, 344, 360, 384, 400, 424, 440, 464, 480}, // 25 frames per 504
// Four receivers - 19 Tx queue events per set of 63, 126, 252, 504  received frames
{3, 6, 10, 13, 16, 20, 23, 26, 30, 33, 36, 40, 43, 46, 50, 53, 56, 59, 62, -1, -1, -1, -1, -1, -1, -1}, // 19 frames per 63
{6, 12, 20, 26, 32, 40, 46, 52, 60, 66, 72, 80, 86, 92, 100, 106, 112, 118, 124, -1, -1, -1, -1, -1, -1, -1}, // 19 frames per 126
{12, 24, 40, 52, 64, 80, 92, 104, 120, 132, 144, 160, 172, 184, 200, 212, 224, 236, 248, -1, -1, -1, -1, -1, -1, -1},  // 19 frames per 252
{24, 48, 80, 104, 128, 160, 184, 208, 240, 264, 288, 320, 344, 368, 400, 424, 448, 472, 496,-1, -1, -1, -1, -1, -1, -1},  // 19 frames per 504
// Five receivers - 15 Tx queue events per set of 63, 126, 252, 504  received frames
{4, 8, 12, 16, 21, 25, 29, 33, 37, 42, 46, 50, 54, 58, 62, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}, // 15 frames per 63
{8, 16, 24, 32, 42, 50, 58, 66, 74, 84, 92, 100, 108, 116, 124, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}, // 15 frames per 126
{ 16, 32, 48, 64, 84, 100, 116, 132, 148, 168, 184, 200, 216, 232, 248, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},  // 15 frames per 252
{ 32, 64, 96, 128, 168, 200, 232, 264, 296, 336, 368, 400, 432, 464, 496, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},  // 15 frames per 504
// Six receivers - 13 Tx queue events per set of 63, 126, 252, 504  received frames
{ 5, 10, 15, 20, 24, 29, 34, 39, 44, 48, 53, 58, 62, -1, -1, -1, -1, -1, -1, -1, -1 , -1, -1, -1, -1, -1}, // 13 frames per 63
{ 10, 20, 30, 40, 48, 58, 68, 78, 88, 96, 106, 116, 124, -1, -1, -1, -1, -1, -1, -1, -1 , -1, -1, -1, -1, -1}, // 13 frames per 126
{ 20, 40, 60, 80, 96, 116, 136, 156, 176, 192, 212, 232, 248, -1, -1, -1, -1, -1, -1, -1, -1 , -1, -1, -1, -1, -1},  // 13 frames per 252
{ 40, 80, 120, 160, 192, 232, 272, 312, 352, 384, 424, 464, 496, -1, -1, -1, -1, -1, -1, -1, -1 , -1, -1, -1, -1, -1},  // 13 frames per 504
//  Seven receivers - 11 Tx queue events per set of 63, 126, 252, 504  received frames
{6, 12, 17, 23, 29, 34, 40, 45, 51, 57, 62, -1, -1, -1, -1, -1, -1, -1 , -1, -1, -1, -1, -1, -1, -1, -1}, // 11 frames per 63
{12, 24, 34, 46, 58, 68, 80, 90, 102, 114, 124, -1, -1, -1, -1, -1, -1, -1 , -1, -1, -1, -1, -1, -1, -1, -1}, // 11 frames per 126
{24, 48, 68, 92, 116, 136, 160, 180, 204, 228, 248, -1, -1, -1, -1, -1, -1, -1 , -1, -1, -1, -1, -1, -1, -1, -1},  // 11 frames per 252
{48, 96, 136, 184, 232, 272, 320, 360, 408, 456, 496, -1, -1, -1, -1, -1, -1, -1 , -1, -1, -1, -1, -1, -1, -1, -1}};  // 11 frames per 504   

void transmitter_save_state(TRANSMITTER *tx) {
  char name[80];
  char value[80];
  int i;

  sprintf(name,"transmitter[%d].channel",tx->channel);
  sprintf(value,"%d",tx->channel);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].alex_antenna",tx->channel);
  sprintf(value,"%d",tx->alex_antenna);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].mic_gain",tx->channel);
  sprintf(value,"%f",tx->mic_gain);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].linein_gain",tx->channel);
  sprintf(value,"%d",tx->linein_gain);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].alc_meter",tx->channel);
  sprintf(value,"%d",tx->alc_meter);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].drive",tx->channel);
  sprintf(value,"%f",tx->drive);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].tune_percent",tx->channel);
  sprintf(value,"%f",tx->tune_percent);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].tune_use_drive",tx->channel);
  sprintf(value,"%d",tx->tune_use_drive);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].attenuation",tx->channel);
  sprintf(value,"%d",tx->attenuation);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].use_rx_filter",tx->channel);
  sprintf(value,"%d",tx->use_rx_filter);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].filter_low",tx->channel);
  sprintf(value,"%d",tx->filter_low);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].filter_high",tx->channel);
  sprintf(value,"%d",tx->filter_high);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].fft_size",tx->channel);
  sprintf(value,"%d",tx->fft_size);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].low_latency",tx->channel);
  sprintf(value,"%d",tx->low_latency);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].buffer_size",tx->channel);
  sprintf(value,"%d",tx->buffer_size);
  setProperty(name,value);
//  sprintf(name,"transmitter[%d].mic_samples",tx->channel);
//  sprintf(value,"%d",tx->mic_samples);
//  setProperty(name,value);
  sprintf(name,"transmitter[%d].mic_sample_rate",tx->channel);
  sprintf(value,"%d",tx->mic_sample_rate);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].mic_dsp_rate",tx->channel);
  sprintf(value,"%d",tx->mic_dsp_rate);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].iq_output_rate",tx->channel);
  sprintf(value,"%d",tx->iq_output_rate);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].output_samples",tx->channel);
  sprintf(value,"%d",tx->output_samples);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].pre_emphasize",tx->channel);
  sprintf(value,"%d",tx->pre_emphasize);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].enable_equalizer",tx->channel);
  sprintf(value,"%d",tx->enable_equalizer);
  setProperty(name,value);
  for(i=0;i<4;i++) {
    sprintf(name,"transmitter[%d].eualizer[%d]",tx->channel,i);
    sprintf(value,"%d",tx->equalizer[i]);
    setProperty(name,value);
  }
  sprintf(name,"transmitter[%d].leveler",tx->channel);
  sprintf(value,"%d",tx->leveler);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].fps",tx->channel);
  sprintf(value,"%d",tx->fps);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].panadapter_low",tx->channel);
  sprintf(value,"%d",tx->panadapter_low);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].panadapter_high",tx->channel);
  sprintf(value,"%d",tx->panadapter_high);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].ctcss_enabled",tx->channel);
  sprintf(value,"%d",tx->ctcss_enabled);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].ctcss",tx->channel);
  sprintf(value,"%d",tx->ctcss);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].eer",tx->channel);
  sprintf(value,"%i",tx->eer);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].eer_amiq",tx->channel);
  sprintf(value,"%i",tx->eer_amiq);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].eer_pgain",tx->channel);
  sprintf(value,"%f",tx->eer_pgain);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].eer_mgain",tx->channel);
  sprintf(value,"%f",tx->eer_mgain);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].eer_pdelay",tx->channel);
  sprintf(value,"%f",tx->eer_pdelay);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].eer_mdelay",tx->channel);
  sprintf(value,"%f",tx->eer_mdelay);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].eer_enable_delays",tx->channel);
  sprintf(value,"%i",tx->eer_enable_delays);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].eer_pwm_min",tx->channel);
  sprintf(value,"%i",tx->eer_pwm_min);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].eer_pwm_max",tx->channel);
  sprintf(value,"%i",tx->eer_pwm_max);
  setProperty(name,value);

  sprintf(name,"transmitter[%d].xit_enabled",tx->channel);
  sprintf(value,"%d",tx->xit_enabled);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].xit",tx->channel);
  sprintf(value,"%ld",tx->xit);
  setProperty(name,value);
  sprintf(name,"transmitter[%d].xit_step",tx->channel);
  sprintf(value,"%ld",tx->xit_step);
  setProperty(name,value);
}

void transmitter_restore_state(TRANSMITTER *tx) {
  char name[80];
  char *value;
  int i;

  sprintf(name,"transmitter[%d].channel",tx->channel);
  value=getProperty(name);
  if(value) tx->channel=atoi(value);
  sprintf(name,"transmitter[%d].alex_antenna",tx->channel);
  value=getProperty(name);
  if(value) tx->alex_antenna=atoi(value);
  sprintf(name,"transmitter[%d].mic_gain",tx->channel);
  value=getProperty(name);
  if(value) tx->mic_gain=atof(value);
  sprintf(name,"transmitter[%d].linein_gain",tx->channel);
  value=getProperty(name);
  if(value) tx->linein_gain=atoi(value);
  sprintf(name,"transmitter[%d].alc_meter",tx->channel);
  value=getProperty(name);
  if(value) tx->alc_meter=atoi(value);
  sprintf(name,"transmitter[%d].drive",tx->channel);
  value=getProperty(name);
  if(value) tx->drive=atof(value);
  sprintf(name,"transmitter[%d].tune_percent",tx->channel);
  value=getProperty(name);
  if(value) tx->tune_percent=atof(value);
  sprintf(name,"transmitter[%d].tune_use_drive",tx->channel);
  value=getProperty(name);
  if(value) tx->tune_use_drive=atoi(value);
  sprintf(name,"transmitter[%d].attenuation",tx->channel);
  value=getProperty(name);
  if(value) tx->attenuation=atoi(value);
  sprintf(name,"transmitter[%d].use_rx_filter",tx->channel);
  value=getProperty(name);
  if(value) tx->use_rx_filter=atoi(value);
  sprintf(name,"transmitter[%d].filter_low",tx->channel);
  value=getProperty(name);
  if(value) tx->filter_low=atoi(value);
  sprintf(name,"transmitter[%d].filter_high",tx->channel);
  value=getProperty(name);
  if(value) tx->filter_high=atoi(value);
  sprintf(name,"transmitter[%d].fft_size",tx->channel);
  value=getProperty(name);
  if(value) tx->fft_size=atoi(value);
  sprintf(name,"transmitter[%d].low_latency",tx->channel);
  value=getProperty(name);
  if(value) tx->low_latency=atoi(value);
  sprintf(name,"transmitter[%d].buffer_size",tx->channel);
  value=getProperty(name);
  if(value) tx->buffer_size=atoi(value);
//  sprintf(name,"transmitter[%d].mic_samples",tx->channel);
//  value=getProperty(name);
//  if(value) tx->mic_samples=atoi(value);
  sprintf(name,"transmitter[%d].mic_sample_rate",tx->channel);
  value=getProperty(name);
  if(value) tx->mic_sample_rate=atoi(value);
  sprintf(name,"transmitter[%d].mic_dsp_rate",tx->channel);
  value=getProperty(name);
  if(value) tx->mic_dsp_rate=atoi(value);
/*
  sprintf(name,"transmitter[%d].iq_output_rate",tx->channel);
  value=getProperty(name);
  if(value) tx->iq_output_rate=atoi(value);
  sprintf(name,"transmitter[%d].output_samples",tx->channel);
  value=getProperty(name);
  if(value) tx->output_samples=atoi(value);
*/
  sprintf(name,"transmitter[%d].pre_emphasize",tx->channel);
  value=getProperty(name);
  if(value) tx->pre_emphasize=atoi(value);
  sprintf(name,"transmitter[%d].enable_equalizer",tx->channel);
  value=getProperty(name);
  if(value) tx->enable_equalizer=atoi(value);
  for(i=0;i<4;i++) {
    sprintf(name,"transmitter[%d].eualizer[%d]",tx->channel,i);
    value=getProperty(name);
    if(value) tx->equalizer[i]=atoi(value);
  }
  sprintf(name,"transmitter[%d].leveler",tx->channel);
  value=getProperty(name);
  if(value) tx->leveler=atoi(value);
  sprintf(name,"transmitter[%d].fps",tx->channel);
  value=getProperty(name);
  if(value) tx->fps=atoi(value);
  sprintf(name,"transmitter[%d].panadapter_low",tx->channel);
  value=getProperty(name);
  if(value) tx->panadapter_low=atoi(value);
  sprintf(name,"transmitter[%d].panadapter_high",tx->channel);
  value=getProperty(name);
  if(value) tx->panadapter_high=atoi(value);
  sprintf(name,"transmitter[%d].ctcss_enabled",tx->channel);
  value=getProperty(name);
  if(value) tx->ctcss_enabled=atoi(value);
  sprintf(name,"transmitter[%d].ctcss",tx->channel);
  value=getProperty(name);
  if(value) tx->ctcss=atoi(value);
  sprintf(name,"transmitter[%d].eer",tx->channel);
  value=getProperty(name);
  if(value) tx->eer=atoi(value);
  sprintf(name,"transmitter[%d].eer_amiq",tx->channel);
  value=getProperty(name);
  if(value) tx->eer_amiq=atoi(value);
  sprintf(name,"transmitter[%d].eer_pgain",tx->channel);
  value=getProperty(name);
  if(value) tx->eer_pgain=atof(value);
  sprintf(name,"transmitter[%d].eer_mgain",tx->channel);
  value=getProperty(name);
  if(value) tx->eer_mgain=atof(value);
  sprintf(name,"transmitter[%d].eer_pdelay",tx->channel);
  value=getProperty(name);
  if(value) tx->eer_pdelay=atof(value);
  sprintf(name,"transmitter[%d].eer_mdelay",tx->channel);
  value=getProperty(name);
  if(value) tx->eer_mdelay=atof(value);
  sprintf(name,"transmitter[%d].eer_enable_delays",tx->channel);
  value=getProperty(name);
  if(value) tx->eer_enable_delays=atoi(value);
  sprintf(name,"transmitter[%d].eer_pwm_min",tx->channel);
  value=getProperty(name);
  if(value) tx->eer_pwm_min=atoi(value);
  sprintf(name,"transmitter[%d].eer_pwm_max",tx->channel);
  value=getProperty(name);
  if(value) tx->eer_pwm_max=atoi(value);

  sprintf(name,"transmitter[%d].xit_enabled",tx->channel);
  value=getProperty(name);
  if(value) tx->xit_enabled=atoi(value);
  sprintf(name,"transmitter[%d].xit",tx->channel);
  value=getProperty(name);
  if(value) tx->xit=atol(value);
  sprintf(name,"transmitter[%d].xit_step",tx->channel);
  value=getProperty(name);
  if(value) tx->xit_step=atol(value);

}

static gboolean update_timer_cb(void *data) {
  int rc;
  double constant1;
  double constant2;
  int fwd_cal_offset=6;

  TRANSMITTER *tx=(TRANSMITTER *)data;
  //if(isTransmitting(radio)) {
    GetPixels(tx->channel,0,tx->pixel_samples,&rc);
    if(rc) {
      update_tx_panadapter(radio);
    }
  //}

  update_mic_level(radio);

  if (tx->tx_info != NULL) update_tx_info(tx);

  if(isTransmitting(radio)) {
    tx->alc=GetTXAMeter(tx->channel,tx->alc_meter);

    int fwd_power;
    int rev_power;
    int ex_power;
    double v1;

    fwd_power=tx->alex_forward_power;
    rev_power=tx->alex_reverse_power;
    if(radio->discovered->device==DEVICE_HERMES_LITE2) {
      ex_power=0;
    } else {
      ex_power=tx->exciter_power;
    }

    switch(radio->discovered->device) {
      case DEVICE_METIS:
        constant1=3.3;
        constant2=0.09;
        break;
      case DEVICE_HERMES:
        constant1=3.3;
        constant2=0.095;
        break;
      case DEVICE_HERMES2:
        constant1=3.3;
        constant2=0.095;
        break;
      case DEVICE_ANGELIA:
        constant1=3.3;
        constant2=0.095;
        break;
      case DEVICE_ORION:
        constant1=5.0;
        constant2=0.108;
        break;
      case DEVICE_ORION2:
        constant1=5.0;
        constant2=0.108;
        break;
      case DEVICE_HERMES_LITE2:
        if(rev_power>fwd_power) {
          fwd_power=tx->alex_reverse_power;
          rev_power=tx->alex_forward_power;
        }
        if (radio->filter_board == HL2_MRF101) {
          // HL2-MFR101 filter board
          constant1 = 3.3;
          constant2 = 0.09;  
          fwd_cal_offset = 6;     
        }
        else {
          // N2ADR hl2 filter board
          constant1=3.3;
          constant2=1.4;  
          fwd_cal_offset=6;      
        }
        break;
    }


    if(fwd_power==0) {
      fwd_power=ex_power;
    }
    
    fwd_power=fwd_power-fwd_cal_offset;
    v1=((double)fwd_power/4095.0)*constant1;
    tx->fwd=(v1*v1)/constant2;    
    
    // Peak detection ring buffer
    tx->fwd_peak = get_peak(tx->fwd_peak_buf, tx->fwd);
        
    if(radio->discovered->device==DEVICE_HERMES_LITE2) {
      tx->exciter=0.0;
    } else {
      ex_power=ex_power-fwd_cal_offset;
      v1=((double)ex_power/4095.0)*constant1;
      tx->exciter=(v1*v1)/constant2;
    }

    tx->rev=0.0;
    if(fwd_power!=0) {
      v1=((double)rev_power/4095.0)*constant1;
      tx->rev=(v1*v1)/constant2;
    }  
  }
  else {
    // Peak detection ring buffer
    tx->fwd_peak = get_peak(tx->fwd_peak_buf, 0);    
  }

  return TRUE;
}

void transmitter_fps_changed(TRANSMITTER *tx) {
  g_source_remove(tx->update_timer_id);
  tx->update_timer_id=g_timeout_add(1000/tx->fps,update_timer_cb,(gpointer)tx);
}

void transmitter_set_ps(TRANSMITTER *tx,gboolean state) {
  tx->puresignal=state;
  if(state) {
    SetPSControl(tx->channel, 0, 0, 1, 0);
  } else {
    SetPSControl(tx->channel, 1, 0, 0, 0);
  }
}

void transmitter_enable_eer(TRANSMITTER *tx,gboolean state) {
  tx->eer=state;
  SetEERRun(0, tx->eer);
}

void transmitter_set_eer_mode_amiq(TRANSMITTER *tx,gboolean state) {
  tx->eer_amiq=state;
  SetEERAMIQ(0, tx->eer_amiq); // 0=phase only, 1=magnitude and phase, 2=magnitude only (not supported here)
}

void transmitter_enable_eer_delays(TRANSMITTER *tx,gboolean state) {
  tx->eer_enable_delays=state;
  SetEERRunDelays(0, tx->eer_enable_delays);
}

void transmitter_set_eer_pgain(TRANSMITTER *tx,gdouble gain) {
  tx->eer_pgain=gain;
  SetEERPgain(0, tx->eer_pgain);
}

void transmitter_set_eer_pdelay(TRANSMITTER *tx,gdouble delay) {
  tx->eer_pdelay=delay;
  SetEERPdelay(0, tx->eer_pdelay/1e6);
}

void transmitter_set_eer_mgain(TRANSMITTER *tx,gdouble gain) {
  tx->eer_mgain=gain;
  SetEERMgain(0, tx->eer_mgain);
}

void transmitter_set_eer_mdelay(TRANSMITTER *tx,gdouble delay) {
  tx->eer_mdelay=delay;
  SetEERMdelay(0, tx->eer_mdelay/1e6);
}

void transmitter_set_twotone(TRANSMITTER *tx,gboolean state) {
g_print("transmitter_set_twotone: %d\n",state);
  tx->ps_twotone=state;
  if(state) {
    SetTXAPostGenMode(tx->channel, 1);
    SetTXAPostGenRun(tx->channel, 1);
  } else {
    SetTXAPostGenRun(tx->channel, 0);
  }
/*
  MOX *m=g_new0(MOX,1);
  m->radio=radio;
  m->state=state;
  g_idle_add(ext_set_mox,(gpointer)m);
*/
  set_mox(radio,state);
}

void transmitter_set_ps_sample_rate(TRANSMITTER *tx,int rate) {
  SetPSFeedbackRate (tx->channel,rate);
}

// Tx packet schedule synched to the rx packets
// Credit to N5EG for most most of the code
// https://github.com/Tom-McDermott/gr-hpsdr/blob/master/lib/HermesProxy.cc
int send_tx_packet_query(TRANSMITTER *tx) {
  tx->packet_counter++;
  
  switch (radio->receivers) {
    case 1: {
      // one Tx frame for each Rx frame
      if(radio->sample_rate == 48000)	return 1;
	    // one Tx frame for each two Rx frames
		  if(radio->sample_rate == 96000)	{
		    if((tx->packet_counter & 0x1) == 0) return 1;
        break;
      }
	
      // one Tx frame for each four Tx frames  
		  if(radio->sample_rate == 192000) {
		    if((tx->packet_counter & 0x3) == 0) return 1;
        break;
      }

      // one Tx frame for each eight Tx frames
		  if(radio->sample_rate == 384000) {
		    if((tx->packet_counter & 0x7) == 0) return 1;
        break;
      }	
    }
    
    case 2: {
      // one Tx frame for each 1.75 Rx frame
      if(radio->sample_rate == 48000)	{
        if(((tx->packet_counter % 0x7) & 0x01) == 0) return 1;
        break;
      }
        
	    // one Tx frame for each 3.5 Rx frames
		  if(radio->sample_rate == 96000)	{
		    if(((tx->packet_counter % 0x7) & 0x03) == 0) return 1;
        break;
      }
	
      // one Tx frame for each four Tx frames  
		  if(radio->sample_rate == 192000) {
		    if((tx->packet_counter % 0x7) == 0) return 1;
        break;
      }

      // one Tx frame for each eight Tx frames
		  if(radio->sample_rate == 384000) {
		    if((tx->packet_counter % 14) == 0) return 1;
        break;
      }	
    }
    
    default: {
      int FrameIndex;
		  int RxNumIndex = radio->receivers-3;	  //   3,  4,  5,  6,  7  -->  0, 1, 2, 3, 4

		  int SpeedIndex = (radio->sample_rate) / 48000;  // 48k, 96k, 192k, 384k -->  1, 2, 4, 8
		  SpeedIndex = SpeedIndex >> 1;		  // 48k, 96k, 192k, 384k -->  0, 1, 2, 4
		  if (SpeedIndex == 4)  SpeedIndex = 3;	  // 48k, 96k, 192k, 384k -->  0, 1, 2, 3

		  int selector = RxNumIndex * 4 + SpeedIndex;	//  0 .. 19

	    // Compute the frame number within a vector
		  if(radio->sample_rate == 48000) FrameIndex = tx->packet_counter % 63;	// FrameIndex is 0..62
	  	if(radio->sample_rate == 96000) FrameIndex = tx->packet_counter % 126;	// FrameIndex is 0..125
	  	if(radio->sample_rate == 192000) FrameIndex = tx->packet_counter % 252;	// FrameIndex is 0..251
	  	if(radio->sample_rate == 384000) FrameIndex = tx->packet_counter % 504;	// FrameIndex is 0..503      
      
      for (int i = 0; i < 26; i++) {
        if (FrameIndex == protocol1_tx_scheduler[selector][i]) return 1; 
      }
      break;
    }   
  }
  return 0;
}

int transmitter_get_mode(TRANSMITTER *tx) {
  gint tx_mode = USB;    
  RECEIVER *tx_receiver = tx->rx;
  if(tx_receiver!=NULL) {
#ifdef USE_VFO_B_MODE_AND_FILTER    
    if(tx_receiver->split) {
      tx_mode = tx_receiver->mode_b;
    } else {
#endif      
      tx_mode = tx_receiver->mode_a;
#ifdef USE_VFO_B_MODE_AND_FILTER      
    }
#endif    
  }
  return tx_mode;
}

#ifdef CWDAEMON
void transmitter_cw_sample_keystate(TRANSMITTER *tx) {
  // Sample the CW daemon key state NOW.
  // This *should* be a steady sample of key state every (1/48 * 126) uS.
  // However, depending on number rx, some gaps between calls to this
  // function have been seen to be 20-30 ms depending on CPU.
  g_mutex_lock(&cwdaemon_mutex); 
  gboolean key_state_now = keytx;
  g_mutex_unlock(&cwdaemon_mutex);   
  

  // key state now is stored and put in a ring buffer. 
  // This works under the assumption this function is called every 2.65 ms
  // (126 p1 samples), we store the sample number at which the key state changed.
  // However, sometimes we may have missed the heartbeat to call this function, so 
  // a ring buffer is used to "catch up".
  // To avoid needing 2 ring buffers (time, key state), sample number is made negative
  // if the key state is OFF. We apply this adjustment to key_up_down_modified now.
  long num_cw_samples = 0;
  int key_up_down_modified;
  if (key_state_now) {
    key_up_down_modified = 1;
  }
  else {
    key_up_down_modified = -1;      
  }
  
  // Has key changed state since last function call?
  if (tx->last_key_state != key_state_now) {
    // Either start the hang time timer (key changed to OFF)
    // or reset the hang time timer (key change to ON)
    set_cwvox(radio, key_state_now);
    
    // Time in ms since the change of key state
    // registered by cwdaemon
    double this_time = read_time_now();
    double cwd_delay = 1E3 * (this_time - cwd_changed_at);
    
    // We send 126 IQ samples to the protocol 1 radio, 
    num_cw_samples = (long)(cwd_delay / (1.0 / 48.0));
    
    // Save key state for next time we call this function
    tx->last_key_state = key_state_now;
    
    // This checks if it has been longer than 2.65 ms since the last 
    // call to this function
    if (num_cw_samples > tx->p1_packet_size){
      // How many "heartbeats" of this function call were missed?
      int num_packets_missed = num_cw_samples / tx->p1_packet_size;
      
      for (int i = 0; i < (num_packets_missed - 1); i++) {     
        // Add the full packets that were missed to the ring buffer
        queue_put(tx->cw_iq_delay_buf, tx->p1_packet_size);
        num_cw_samples -= tx->p1_packet_size;
      }
      // Finally last packet (where key state change occured less than 2.65 ms ago)
      queue_put(tx->cw_iq_delay_buf, (key_up_down_modified * tx->p1_packet_size));        
    }
    else {
      // State change occured in last 2.65 ms
      long to_store = (key_up_down_modified * num_cw_samples);
      queue_put(tx->cw_iq_delay_buf, to_store);
    }
  }
  else {
    // Key state hasn't changed
    if (key_state_now) {      
      queue_put(tx->cw_iq_delay_buf, key_up_down_modified - 1);           
    } 
    else {
      queue_put(tx->cw_iq_delay_buf, key_up_down_modified);         
    }      
  }
}
#endif


// Protocol 1 receive thread calls this, to send 126 iq samples in a 
// tx packet
void full_tx_buffer(TRANSMITTER *tx, gboolean force_send) {
  if (!isTransmitting(radio) && radio->discovered->device!=DEVICE_HERMES_LITE2) return;
  if ((isTransmitting(radio)) && (radio->classE)) return;
  
  // Work out if we are going to send a tx packet or return
  
  if (force_send) {
    g_print("Force %d\n", radio->hl2->overflow);
  }
  else {
    if (!send_tx_packet_query(tx)) return;
  }
  
  
  #ifdef CWDAEMON
  // PC generated cw waveform in the IQ packet sent the protocol 1 radio
  gint tx_mode = transmitter_get_mode(tx);    
  
  if (((radio->cwdaemon) && (tx_mode==CWL || tx_mode==CWU)) && (radio->cw_generation_mode == CWGEN_PC)) {
    transmitter_cw_sample_keystate(tx);
  }  

  // Find out the key state (and implied time of key state change)
  // after is it run through the delay line/ringbuffer  
  bool key_state_delayed = FALSE;  
  long this_sample_cw_idx = -126;      

  if ((radio->cwdaemon) && (tx_mode==CWL || tx_mode==CWU)) {    
    int rv = queue_get(tx->cw_iq_delay_buf, &this_sample_cw_idx);
    if (rv != 0) g_print("cw waveform queue error = %d\n", rv);
    
    if (this_sample_cw_idx < 0) {
      key_state_delayed = FALSE;
      // We have stored the sample as a negative number to allow us to know
      // this is for a key off, we now need it +ve to make the number useable
      // further on in this function.
      // If we want the whole packet to be 0, we keep the -ve number
      if (this_sample_cw_idx != -(tx->p1_packet_size)) {
        this_sample_cw_idx = (this_sample_cw_idx * -1);    
      }
      // Check if key has been off for longer than hang time
      if ((radio->mox) && (key_state_delayed == FALSE)) update_cwvox(radio);         
    }
    else {
      key_state_delayed = TRUE;
    }
  }
  #endif
  
  int rv = 0;
  int buf_underflow = 0;
  
  for (int j = 0; j < tx->p1_packet_size; j++) {  
    long isample = 0;
    long qsample = 0;           
    
    
  
    g_mutex_lock((&tx->queue_mutex));
    rv = queue_get(tx->p1_ringbuf, &isample);
    if (rv < 0) buf_underflow++;
    rv = queue_get(tx->p1_ringbuf, &qsample);   
    if (rv < 0) buf_underflow++;    
    g_mutex_unlock((&tx->queue_mutex));
    
    #ifdef CWDAEMON
    if (((radio->cwdaemon) && (tx_mode==CWL || tx_mode==CWU)) && (radio->cw_generation_mode == CWGEN_PC)) {
      isample = 0;
      // Send the buffered (delayed) cw waveform.
      if(key_state_delayed) { 
        qsample = 0;
        if (j >= this_sample_cw_idx) {
          // Transition from 0 to 1 in this packet
          qsample = floor(32767.0 * cw_waveform[tx->cw_waveform_idx] + 0.5);
          if (tx->cw_waveform_idx < CW_WAVEFORM_SAMPLES - 1) tx->cw_waveform_idx++;
        }
      }
      else {
        if (j < this_sample_cw_idx) {  
          // Transition from 1 to 0 in this packet (but might still be 
          // on the rise of the waveform, so increment idx still     
          qsample = floor(32767.0 * cw_waveform[tx->cw_waveform_idx] + 0.5);
          if (tx->cw_waveform_idx < CW_WAVEFORM_SAMPLES - 1) tx->cw_waveform_idx++;      
        }
        else {
          // Key off, either on the fall of the waveform (or already at 0)
          qsample = floor(32767.0 * cw_waveform[tx->cw_waveform_idx] + 0.5);
          if (tx->cw_waveform_idx > 0) tx->cw_waveform_idx--;        
        }
      }      
    }
    #endif
    
    protocol1_iq_samples(isample, qsample);
  }
  //if (buf_underflow > 0) g_print("buff\n");
}


// We have 1024 samples, now going to exchange them for 1024 iq samples
// then put them into the ring buffer
void full_tx_buffer_process(TRANSMITTER *tx) {
  long isample;
  long qsample;
  long lsample;
  long rsample;
  double gain;
  int j;
  int error;
  
  // round half towards zero  
  #define ROUNDHTZ(x) ((x)>=0.0?(long)floor((x)*gain+0.5):(long)ceil((x)*gain-0.5))  

  switch(radio->discovered->protocol) {
#ifdef RADIOBERRY
    case RADIOBERRY_PROTOCOL:
#endif
    case PROTOCOL_1:
      gain=32767.0;  // 16 bit
      break;
    case PROTOCOL_2:
      gain=8388607.0; // 24 bit
      break;
#ifdef SOAPYSDR
    case PROTOCOL_SOAPYSDR:
      gain=32767.0;  // 16 bit
      break;
#endif
  }
  
  update_vox(radio);
  fexchange0(tx->channel, tx->mic_input_buffer, tx->iq_output_buffer, &error);
  if(error!=0) {
    fprintf(stderr,"full_tx_buffer_process: channel=%d fexchange0: error=%d\n",tx->channel,error);
  }

  Spectrum0(1, tx->channel, 0, 0, tx->iq_output_buffer);
  
  if ((radio->discovered->protocol == PROTOCOL_1) && (!radio->classE)) {
    // not going to send out packets now, put them in the ring buffer
    // then every rx packet, we send a tx packet @48k  
    for(int j = 0; j < tx->output_samples; j++) {
      long isample = ROUNDHTZ(tx->iq_output_buffer[j*2]);
      long qsample = ROUNDHTZ(tx->iq_output_buffer[(j*2)+1]);  
      g_mutex_lock((&tx->queue_mutex));    
      queue_put(tx->p1_ringbuf, isample);
      queue_put(tx->p1_ringbuf, qsample);    
      g_mutex_unlock(&tx->queue_mutex);
    }
    return;
  }
  
  if(isTransmitting(radio)) {
    if(radio->classE) {
      for(j=0;j<tx->output_samples;j++) {
        tx->inI[j]=tx->iq_output_buffer[j*2];
        tx->inQ[j]=tx->iq_output_buffer[(j*2)+1];
      }
      // EER processing
      // input is TX IQ samples in inI,inQ
      // output phase/IQ/magnitude is written back to inI,inQ and
      //   outMI,outMQ contain the scaled and delayed input samples
      
      xeerEXTF(0, tx->inI, tx->inQ, tx->inI, tx->inQ, tx->outMI, tx->outMQ, isTransmitting(radio), tx->output_samples);

    }

    for(j=0;j<tx->output_samples;j++) {
      if(radio->classE) {
        isample=ROUNDHTZ(tx->inI[j]);
        qsample=ROUNDHTZ(tx->inQ[j]);
        lsample=ROUNDHTZ(tx->outMI[j]);
        rsample=ROUNDHTZ(tx->outMQ[j]);
      } else {
        if(radio->iqswap) {
          qsample=ROUNDHTZ(tx->iq_output_buffer[j*2]);
          isample=ROUNDHTZ(tx->iq_output_buffer[(j*2)+1]);
        } else {
          isample=ROUNDHTZ(tx->iq_output_buffer[j*2]);
          qsample=ROUNDHTZ(tx->iq_output_buffer[(j*2)+1]);
        }
      }
      switch(radio->discovered->protocol) {
        case PROTOCOL_1:
          if(radio->classE) {
            protocol1_eer_iq_samples(isample,qsample,lsample,rsample);
          } else {
            // Unreachable code, protocol1 and not class E
            // has returned above.
            return;
          }
          break;
        case PROTOCOL_2:
          protocol2_iq_samples(isample,qsample);
          break;
#ifdef SOAPYSDR
        case PROTOCOL_SOAPYSDR:
          soapy_protocol_iq_samples((float)isample,(float)qsample);
          break;
#endif
      }
    }
  }
#undef ROUNDHTZ
}

void add_mic_sample(TRANSMITTER *tx,float mic_sample) {
  int mode;
  double mic_sample_double;
 
  if(tx->rx!=NULL) {
    mode= transmitter_get_mode(tx);

    if(mode==CWL || mode==CWU || radio->tune) {
      mic_sample_double=0.0;
    } else {
      mic_sample_double=(double)mic_sample;
    }
    tx->mic_input_buffer[tx->mic_samples*2]=mic_sample_double;
    tx->mic_input_buffer[(tx->mic_samples*2)+1]=0.0; //mic_sample_double;
    tx->mic_samples++;
   
    if(tx->mic_samples==tx->buffer_size) {
      full_tx_buffer_process(tx);
      tx->mic_samples=0;
    }
  }
}

void transmitter_set_filter(TRANSMITTER *tx,int low,int high) {

  gint mode = transmitter_get_mode(tx);

  if(tx->use_rx_filter) {
    RECEIVER *rx=tx->rx;
    FILTER *mode_filters=filters[mode];
    FILTER *filter=&mode_filters[F5];
    if(rx!=NULL) {
      //if(rx->split) {
      //  filter=&mode_filters[rx->filter_b];
      //} else {
        filter=&mode_filters[rx->filter_a];
      //}
    }
    if(mode==CWL) {
      tx->actual_filter_low=-radio->cw_keyer_sidetone_frequency-filter->low;
      tx->actual_filter_high=-radio->cw_keyer_sidetone_frequency+filter->high;
    } else if(mode==CWU) {
      tx->actual_filter_low=radio->cw_keyer_sidetone_frequency-filter->low;
      tx->actual_filter_high=radio->cw_keyer_sidetone_frequency+filter->high;
    } else {
      tx->actual_filter_low=filter->low;
      tx->actual_filter_high=filter->high;
    }
  } else {
    switch(mode) {
      case LSB:
      case CWL:
      case DIGL:
        tx->actual_filter_low=-high;
        tx->actual_filter_high=-low;
        break;
      case USB:
      case CWU:
      case DIGU:
        tx->actual_filter_low=low;
        tx->actual_filter_high=high;
        break;
      case DSB:
      case AM:
      case SAM:
        tx->actual_filter_low=-high;
        tx->actual_filter_high=high;
        break;
      case FMN:
        if(tx->deviation==2500) {
          tx->actual_filter_low=-4000;
          tx->actual_filter_high=4000;
        } else {
          tx->actual_filter_low=-8000;
          tx->actual_filter_high=8000;
        }
        break;
      case DRM:
        tx->actual_filter_low=7000;
        tx->actual_filter_high=17000;
        break;
    }
  }
  double fl=tx->actual_filter_low;
  double fh=tx->actual_filter_high;

  SetTXABandpassFreqs(tx->channel, fl,fh);

  if(tx->panadapter != NULL) {
    update_tx_panadapter(radio);
  }
}

void transmitter_set_mode(TRANSMITTER* tx,int mode) {
  if(tx!=NULL) {
    SetTXAMode(tx->channel, mode);
    transmitter_set_filter(tx,tx->filter_low,tx->filter_high);
  }
}

void transmitter_set_deviation(TRANSMITTER *tx) {
  SetTXAFMDeviation(tx->channel, (double)tx->deviation);
}

void transmitter_set_am_carrier_level(TRANSMITTER *tx) {
  SetTXAAMCarrierLevel(tx->channel, tx->am_carrier_level);
}

void transmitter_set_ctcss(TRANSMITTER *tx,gboolean run,int i) {
  tx->ctcss_enabled=run;
  tx->ctcss=i;
  SetTXACTCSSFreq(tx->channel, ctcss_frequencies[tx->ctcss]);
  SetTXACTCSSRun(tx->channel, tx->ctcss_enabled);
}

void transmitter_set_pre_emphasize(TRANSMITTER *tx,int state) {
  SetTXAFMEmphPosition(tx->channel,state);
}

static void create_visual(TRANSMITTER *tx) {
  tx->panadapter=create_tx_panadapter(tx);
}

void transmitter_init_analyzer(TRANSMITTER *tx) {
    int flp[] = {0};
    double keep_time = 0.1;
    int n_pixout=1;
    int spur_elimination_ffts = 1;
    int data_type = 1;
    int fft_size = 8192;
    int window_type = 4;
    double kaiser_pi = 14.0;
    int overlap = 2048;
    int clip = 0;
    int span_clip_l = 0;
    int span_clip_h = 0;
    int pixels=tx->pixels;
    int stitches = 1;
    int calibration_data_set = 0;
    double span_min_freq = 0.0;
    double span_max_freq = 0.0;

    g_print("transmitter_init_analyzer: width=%d pixels=%d\n",tx->panadapter_width,tx->pixels);

    if(tx->pixel_samples!=NULL) {
      g_free(tx->pixel_samples);
      tx->pixel_samples=NULL;
    }

    if(tx->pixels>0) {
      tx->pixel_samples=g_new0(float,tx->pixels);
      int max_w = fft_size + (int) fmin(keep_time * (double) tx->fps, keep_time * (double) fft_size * (double) tx->fps);

      overlap = (int)max(0.0, ceil(fft_size - (double)tx->mic_sample_rate / (double)tx->fps));

      fprintf(stderr,"SetAnalyzer id=%d buffer_size=%d overlap=%d\n",tx->channel,tx->output_samples,overlap);


      SetAnalyzer(tx->channel,
            n_pixout,
            spur_elimination_ffts, //number of LO frequencies = number of ffts used in elimination
            data_type, //0 for real input data (I only); 1 for complex input data (I & Q)
            flp, //vector with one elt for each LO frequency, 1 if high-side LO, 0 otherwise
            fft_size, //size of the fft, i.e., number of input samples
            tx->output_samples, //number of samples transferred for each OpenBuffer()/CloseBuffer()
            window_type, //integer specifying which window function to use
            kaiser_pi, //PiAlpha parameter for Kaiser window
            overlap, //number of samples each fft (other than the first) is to re-use from the previous
            clip, //number of fft output bins to be clipped from EACH side of each sub-span
            span_clip_l, //number of bins to clip from low end of entire span
            span_clip_h, //number of bins to clip from high end of entire span
            pixels, //number of pixel values to return.  may be either <= or > number of bins
            stitches, //number of sub-spans to concatenate to form a complete span
            calibration_data_set, //identifier of which set of calibration data to use
            span_min_freq, //frequency at first pixel value8192
            span_max_freq, //frequency at last pixel value
            max_w //max samples to hold in input ring buffers
      );
  }

}

TRANSMITTER *create_transmitter(int channel) {
  gint rc;
g_print("create_transmitter: channel=%d\n",channel);
  TRANSMITTER *tx=g_new0(TRANSMITTER,1);
  tx->channel=channel;
  tx->dac=0;
  tx->alex_antenna=ALEX_TX_ANTENNA_1;
  tx->mic_gain=0.0;
  tx->rx=NULL;

  g_mutex_init(&tx->queue_mutex);

  tx->alc_meter=TXA_ALC_PK;
  tx->exciter_power=0;
  tx->alex_forward_power=0;
  tx->alex_reverse_power=0;
  tx->swr = 1.0;
  tx->fwd_peak = 0;
  tx->fwd = 0;
  tx->rev = 0;
  
  tx->eer_amiq=1;
  tx->eer_pgain=0.5;
  tx->eer_mgain=0.5;
  tx->eer_pdelay=200;
  tx->eer_mdelay=200;
  tx->eer_enable_delays=TRUE;
  tx->eer_pwm_min=100;
  tx->eer_pwm_max=800;

  tx->use_rx_filter=FALSE;
  tx->filter_low=150;
  tx->filter_high=2850;

  tx->actual_filter_low=tx->filter_low;
  tx->actual_filter_high=tx->filter_high;

  tx->fft_size=2048;
  tx->low_latency=FALSE;

  switch(radio->discovered->protocol) {
    case PROTOCOL_1:
      tx->mic_sample_rate=48000;
      tx->mic_dsp_rate=48000;
      tx->iq_output_rate=48000;
      tx->buffer_size=1024;
      tx->output_samples=1024;
      tx->p1_packet_size = 126;
      tx->packet_counter = 0;
      break;
    case PROTOCOL_2:
      tx->mic_sample_rate=48000;
      tx->mic_dsp_rate=96000;
      tx->iq_output_rate=192000;
      tx->buffer_size=1024;
      tx->output_samples=1024*(tx->iq_output_rate/tx->mic_sample_rate);
      break;
#ifdef SOAPYSDR
    case PROTOCOL_SOAPYSDR:
      tx->mic_sample_rate=48000;
      tx->mic_dsp_rate=96000;
      tx->iq_output_rate=radio->sample_rate;
      tx->buffer_size=1024;
      tx->output_samples=1024*(tx->iq_output_rate/tx->mic_sample_rate);
      break;
#endif
  }

  tx->mic_samples=0;
  tx->mic_input_buffer=g_new(gdouble,2*tx->buffer_size);
  tx->iq_output_buffer=g_new(gdouble,2*tx->output_samples);

  // EER buffers
  tx->inI=g_new(gfloat,tx->output_samples);
  tx->inQ=g_new(gfloat,tx->output_samples);
  tx->outMI=g_new(gfloat,tx->output_samples);
  tx->outMQ=g_new(gfloat,tx->output_samples);

  tx->pre_emphasize=FALSE;
  tx->enable_equalizer=FALSE;
  tx->equalizer[0]=tx->equalizer[1]=tx->equalizer[2]=tx->equalizer[3]=0;
  tx->leveler=FALSE;

  tx->ctcss_enabled=FALSE;
  tx->ctcss=11;
  tx->tone_level=0.2;

  tx->deviation=2500.0;

  tx->compressor=FALSE;
  tx->compressor_level=0.0;

  tx->am_carrier_level=0.5;

  tx->fps=10;
  tx->pixels=0;
  tx->pixel_samples=NULL;

  tx->drive=20.0;
  tx->tune_use_drive=FALSE;
  tx->tune_percent=10.0;

  tx->temperature = 0.0;

  tx->panadapter_high=20;
  tx->panadapter_low=-140;
  tx->panadapter=NULL;
  tx->panadapter_surface=NULL;

  tx->puresignal=FALSE;
  tx->ps_twotone=FALSE;
  tx->ps_feedback=FALSE;
  tx->ps_auto=TRUE;
  tx->ps_single=FALSE;

  tx->dialog=NULL;

  tx->xit_enabled=FALSE;
  tx->xit=0;
  tx->xit_step=1000;

  tx->updated=FALSE;

  // 40 packets of 126 samples = 5040 = approx 106 ms delay
  // Has to be able to cope with large dumps from pulseaudio
  tx->p1_ringbuf = create_long_ringbuffer(7500, 0);
  #ifdef CWDAEMON
  tx->cw_waveform_idx = 0;
  // Approx 13 ms delay between PTT and CW waveform
  // This should be changed in future version to be adjustable
  tx->cw_iq_delay_buf = create_long_ringbuffer(5, -126);
  tx->last_key_state = FALSE;
  #endif

  gint mode=USB;

  transmitter_restore_state(tx);

  OpenChannel(tx->channel,
              tx->buffer_size,
              2048, // tx->fft_size,
              tx->mic_sample_rate,
              tx->mic_dsp_rate,
              tx->iq_output_rate,
              1, // transmit
              0, // run
              0.010, 0.025, 0.0, 0.010, 0);

  TXASetNC(tx->channel, tx->fft_size);
  TXASetMP(tx->channel, tx->low_latency);

  SetTXABandpassWindow(tx->channel, 1);
  SetTXABandpassRun(tx->channel, 1);

  SetTXAFMEmphPosition(tx->channel,tx->pre_emphasize);

  SetTXACFIRRun(tx->channel, 0);
  SetTXAGrphEQ(tx->channel, tx->equalizer);
  if(tx->enable_equalizer) {
    SetTXAEQRun(tx->channel, 1);
  } else {
    SetTXAEQRun(tx->channel, 0);
  }

  transmitter_set_ctcss(tx,tx->ctcss_enabled,tx->ctcss);
  SetTXAAMSQRun(tx->channel, 0);
  SetTXAosctrlRun(tx->channel, 0);

  SetTXAALCAttack(tx->channel, 1);
  SetTXAALCDecay(tx->channel, 10);
  SetTXAALCSt(tx->channel, 1); // turn it on (always on)

  SetTXALevelerAttack(tx->channel, 1);
  SetTXALevelerDecay(tx->channel, 500);
  SetTXALevelerTop(tx->channel, 5.0);
  SetTXALevelerSt(tx->channel, tx->leveler);

  SetTXAPreGenMode(tx->channel, 0);
  SetTXAPreGenToneMag(tx->channel, 0.0);
  SetTXAPreGenToneFreq(tx->channel, 0.0);
  SetTXAPreGenRun(tx->channel, 0);

  SetTXAPostGenMode(tx->channel, 0);
  SetTXAPostGenToneMag(tx->channel, tx->tone_level);
  SetTXAPostGenTTMag(tx->channel, tx->tone_level,tx->tone_level);
  SetTXAPostGenToneFreq(tx->channel, 0.0);
  SetTXAPostGenRun(tx->channel, 0);

  SetTXAPanelGain1(tx->channel,pow(10.0, tx->mic_gain / 20.0));
  SetTXAPanelRun(tx->channel, 1);

  SetTXAFMDeviation(tx->channel, tx->deviation);
  SetTXAAMCarrierLevel(tx->channel, tx->am_carrier_level);

  SetTXACompressorGain(tx->channel, tx->compressor_level);
  SetTXACompressorRun(tx->channel, tx->compressor);

  create_eerEXT(0, // id
                0, // run
                tx->buffer_size, // size
                48000, // rate
                tx->eer_mgain, // mgain
                tx->eer_pgain, // pgain
                tx->eer_enable_delays, // rundelays
                tx->eer_mdelay/1e6, // mdelay
                tx->eer_pdelay/1e6, // pdelay
                tx->eer_amiq); // amiq

  SetEERRun(0, 1);

  transmitter_set_mode(tx,mode);

  tx->tx_info = NULL;
  tx->num_tx_info_meters = 0; 
  
  for (int i = 0; i <= NUM_TX_METERS; i++) {
    tx->tx_info_meter[i] = NULL;
  }
  tx->tx_info_meter[NUM_TX_METERS+1] = NULL;  
  
  tx->fwd_peak_buf = create_peak_detector(PEAK_DETECT_BUF_SIZE, 0);
  
  create_visual(tx);

  XCreateAnalyzer(tx->channel, &rc, 262144, 1, 1, "");
  if (rc != 0) {
    fprintf(stderr, "XCreateAnalyzer channel=%d failed: %d\n",tx->channel,rc);
  } else {
    transmitter_init_analyzer(tx);
g_print("update_timer: fps=%d\n",tx->fps);
    tx->update_timer_id=g_timeout_add(1000/tx->fps,update_timer_cb,(gpointer)tx);
  }

  switch(radio->discovered->protocol) {
    case PROTOCOL_1:
      break;
    case PROTOCOL_2:
      break;
#ifdef SOAPYSDR
    case PROTOCOL_SOAPYSDR:
      soapy_protocol_create_transmitter(tx);
      soapy_protocol_set_tx_antenna(tx,radio->dac[0].antenna);
      soapy_protocol_set_tx_gain(&radio->dac[0]);
      soapy_protocol_set_tx_frequency(tx);
      soapy_protocol_start_transmitter(tx);
      break;
#endif
  }

  /*
  if(radio->local_microphone) {
    if(audio_open_input(radio)<0) {
      radio->local_microphone=FALSE;
    }
  }
  */

  return tx;
}

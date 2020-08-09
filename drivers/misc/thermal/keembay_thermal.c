// SPDX-License-Identifier: GPL-2.0-only
/*
 * keembay-thermal.c - KeemBay Thermal Driver.
 *
 * Copyright (C) 2019-2020 Intel Corporation
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include "keembay_tsens.h"
/* Register values for keembay temperature (PVT Sensor) */
#define AON_TSENS_TRIM0_CFG 0x0030
#define AON_TSENS_TRIM1_CFG 0x0034
#define AON_TSENS_CFG 0x0038
#define AON_TSENS_INT0 0x203c
#define AON_TSENS_INT1 0x2040
#define AON_TSENS_IRQ_CLEAR 0x0044
#define AON_TSENS_DATA0 0x0048
#define MSS_T_SAMPLE_VALID 0x80000000
#define MSS_T_SAMPLE 0x3ff
#define CSS_T_SAMPLE_VALID 0x8000
#define CSS_T_SAMPLE 0x3ff
#define NCE1_T_SAMPLE_VALID 0x80000000
#define NCE1_T_SAMPLE 0x3ff
#define NCE0_T_SAMPLE_VALID 0x8000
#define NCE0_T_SAMPLE 0x3ff
#define AON_TSENS_DATA1 0x004c
#define AON_INTERFACE 0x20260000
/* Bit shift for registers*/
#define MSS_BIT_SHIFT 16
#define CSS_BIT_SHIFT 0
#define NCE0_BIT_SHIFT 0
#define NCE1_BIT_SHIFT 16
/* mask values for config register */
#define CFG_MASK_AUTO 0x80ff //(auto configuration)
#define CFG_IRQ_MASK 0x8fff
#define CFG_MASK_MANUAL  0x000f // TSENS_EN (manual configuration)
/* temperature boundary cases */
#define Lower_Temp_Nrange 27
#define Upper_Temp_Nrange 771
#define Lower_Temp -39956
#define Upper_Temp 125025

/* temperature calculation lookup */
struct raw_kmb {
	int N;
	int temp;
};
static struct raw_kmb raw_kmb_data[] = {
{27, -39956}, {28, -39637}, {29, -39319}, {30, -39001}, {31, -38684},

{32, -38367}, {33, -38050}, {34, -37734}, {35, -37418}, {36, -37103},

{37, -36787}, {38, -36472}, {39, -36158}, {40, -35844}, {41, -35530},

{42, -35216}, {43, -34903}, {44, -34590}, {45, -34278}, {46, -33966},

{47, -33654}, {48, -33343}, {49, -33032}, {50, -32721}, {51, -32411},

{52, -32101}, {53, -31791}, {54, -31482}, {55, -31173}, {56, -30864},

{57, -30556}, {58, -30248}, {59, -29940}, {60, -29633}, {61, -29326},

{62, -29020}, {63, -28713}, {64, -28407}, {65, -28102}, {66, -27797},

{67, -27492}, {68, -27187}, {69, -26883}, {70, -26579}, {71, -26276},

{72, -25973}, {73, -25670}, {74, -25367}, {75, -25065}, {76, -24763},

{77, -24462}, {78, -24160}, {79, -23860}, {80, -23559}, {81, -23259},

{82, -22959}, {83, -22660}, {84, -22360}, {85, -22062}, {86, -21763},

{87, -21465}, {88, -21167}, {89, -20869}, {90, -20572}, {91, -20275},

{92, -19979}, {93, -19683}, {94, -19387}, {95, -19091}, {96, -18796},

{97, -18501}, {98, -18206}, {99, -17912}, {100, -17618}, {101, -17325},

{102, -17031}, {103, -16738}, {104, -16446}, {105, -16153}, {106, -15861},

{107, -15570}, {108, -15278}, {109, -14987}, {110, -14697}, {111, -14406},

{112, -14116}, {113, -13826}, {114, -13537}, {115, -13248}, {116, -12959},

{117, -12670}, {118, -12382}, {119, -12094}, {120, -11807}, {121, -11520},

{122, -11233}, {123, -10946}, {124, -10660}, {125, -10374}, {126, -10088},

{127, -9803}, {128, -9518}, {129, -9233}, {130, -8949}, {131, -8665},

{132, -8381}, {133, -8097}, {134, -7814}, {135, -7531}, {136, -7249},

{137, -6967}, {138, -6685}, {139, -6403}, {140, -6122}, {141, -5841},

{142, -5560}, {143, -5279}, {144, -4999}, {145, -4720}, {146, -4440},

{147, -4161}, {148, -3882}, {149, -3603}, {150, -3325}, {151, -3047},

{152, -2770}, {153, -2492}, {154, -2215}, {155, -1938}, {156, -1662},

{157, -1386}, {158, -1110}, {159, -834}, {160, -559}, {161, -284},

{162, -9}, {163, 265}, {164, 539}, {165, 813}, {166, 1086},

{167, 1360}, {168, 1633}, {169, 1905}, {170, 2177}, {171, 2449},

{172, 2721}, {173, 2993}, {174, 3264}, {175, 3535}, {176, 3805},

{177, 4075}, {178, 4345}, {179, 4615}, {180, 4884}, {181, 5153},

{182, 5422}, {183, 5691}, {184, 5959}, {185, 6227}, {186, 6495},

{187, 6762}, {188, 7029}, {189, 7296}, {190, 7562}, {191, 7829},

{192, 8095}, {193, 8360}, {194, 8626}, {195, 8891}, {196, 9155},

{197, 9420}, {198, 9684}, {199, 9948}, {200, 10212}, {201, 10475},

{202, 10738}, {203, 11001}, {204, 11264}, {205, 11526}, {206, 11788},

{207, 12049}, {208, 12311}, {209, 12572}, {210, 12833}, {211, 13093},

{212, 13354}, {213, 13614}, {214, 13874}, {215, 14133}, {216, 14392},

{217, 14651}, {218, 14910}, {219, 15168}, {220, 15426}, {221, 15684},

{222, 15942}, {223, 16199}, {224, 16456}, {225, 16713}, {226, 16969},

{227, 17225}, {228, 17481}, {229, 17737}, {230, 17992}, {231, 18247},

{232, 18502}, {233, 18757}, {234, 19011}, {235, 19265}, {236, 19519},

{237, 19772}, {238, 20025}, {239, 20278}, {240, 20531}, {241, 20784},

{242, 21036}, {243, 21288}, {244, 21539}, {245, 21791}, {246, 22042},

{247, 22292}, {248, 22543}, {249, 22793}, {250, 23043}, {251, 23293},

{252, 23543}, {253, 23792}, {254, 24041}, {255, 24290}, {256, 24538},

{257, 24786}, {258, 25034}, {259, 25282}, {260, 25529}, {261, 25776},

{262, 26023}, {263, 26270}, {264, 26516}, {265, 26763}, {266, 27008},

{267, 27254}, {268, 27499}, {269, 27745}, {270, 27989}, {271, 28234},

{272, 28478}, {273, 28722}, {274, 28966}, {275, 29210}, {276, 29453},

{277, 29696}, {278, 29939}, {279, 30182}, {280, 30424}, {281, 30666},

{282, 30908}, {283, 31149}, {284, 31391}, {285, 31632}, {286, 31873},

{287, 32113}, {288, 32353}, {289, 32593}, {290, 32833}, {291, 33073},

{292, 33312}, {293, 33551}, {294, 33790}, {295, 34029}, {296, 34267},

{297, 34505}, {298, 34743}, {299, 34980}, {300, 35218}, {301, 35455},

{302, 35692}, {303, 35928}, {304, 36165}, {305, 36401}, {306, 36637},

{307, 36872}, {308, 37108}, {309, 37343}, {310, 37578}, {311, 37813},

{312, 38047}, {313, 38281}, {314, 38515}, {315, 38749}, {316, 38982},

{317, 39216}, {318, 39448}, {319, 39681}, {320, 39914}, {321, 40146},

{322, 40378}, {323, 40610}, {324, 40841}, {325, 41073}, {326, 41304},

{327, 41535}, {328, 41765}, {329, 41996}, {330, 42226}, {331, 42456},

{332, 42686}, {333, 42915}, {334, 43144}, {335, 43373}, {336, 43602},

{337, 43830}, {338, 44059}, {339, 44287}, {340, 44515}, {341, 44742},

{342, 44970}, {343, 45197}, {344, 45424}, {345, 45650}, {346, 45877},

{347, 46103}, {348, 46329}, {349, 46555}, {350, 46780}, {351, 47006},

{352, 47231}, {353, 47456}, {354, 47680}, {355, 47905}, {356, 48129},

{357, 48353}, {358, 48576}, {359, 48800}, {360, 49023}, {361, 49246},

{362, 49469}, {363, 49692}, {364, 49914}, {365, 50136}, {366, 50358},

{367, 50580}, {368, 50801}, {369, 51023}, {370, 51244}, {371, 51464},

{372, 51685}, {373, 51905}, {374, 52126}, {375, 52346}, {376, 52565},

{377, 52785}, {378, 53004}, {379, 53223}, {380, 53442}, {381, 53661},

{382, 53879}, {383, 54097}, {384, 54315}, {385, 54533}, {386, 54750},

{387, 54968}, {388, 55185}, {389, 55402}, {390, 55618}, {391, 55835},

{392, 56051}, {393, 56267}, {394, 56483}, {395, 56699}, {396, 56914},

{397, 57129}, {398, 57344}, {399, 57559}, {400, 57773}, {401, 57988},

{402, 58202}, {403, 58416}, {404, 58630}, {405, 58843}, {406, 59056},

{407, 59269}, {408, 59482}, {409, 59695}, {410, 59907}, {411, 60120},

{412, 60332}, {413, 60543}, {414, 60755}, {415, 60966}, {416, 61178},

{417, 61389}, {418, 61599}, {419, 61810}, {420, 62020}, {421, 62231},

{422, 62440}, {423, 62650}, {424, 62860}, {425, 63069}, {426, 63278},

{427, 63487}, {428, 63696}, {429, 63904}, {430, 64113}, {431, 64321},

{432, 64529}, {433, 64737}, {434, 64944}, {435, 65151}, {436, 65358},

{437, 65565}, {438, 65772}, {439, 65979}, {440, 66185}, {441, 66391},

{442, 66597}, {443, 66803}, {444, 67008}, {445, 67213}, {446, 67419},

{447, 67624}, {448, 67828}, {449, 68033}, {450, 68237}, {451, 68441},

{452, 68645}, {453, 68849}, {454, 69052}, {455, 69256}, {456, 69459},

{457, 69662}, {458, 69865}, {459, 70067}, {460, 70270}, {461, 70472},

{462, 70674}, {463, 70876}, {464, 71077}, {465, 71279}, {466, 71480},

{467, 71681}, {468, 71882}, {469, 72082}, {470, 72283}, {471, 72483},

{472, 72683}, {473, 72883}, {474, 73083}, {475, 73282}, {476, 73481},

{477, 73680}, {478, 73879}, {479, 74078}, {480, 74277}, {481, 74475},

{482, 74673}, {483, 74871}, {484, 75069}, {485, 75266}, {486, 75464},

{487, 75661}, {488, 75858}, {489, 76055}, {490, 76252}, {491, 76448},

{492, 76644}, {493, 76841}, {494, 77037}, {495, 77232}, {496, 77428},

{497, 77623}, {498, 77818}, {499, 78013}, {500, 78208}, {501, 78403},

{502, 78597}, {503, 78792}, {504, 78986}, {505, 79180}, {506, 79373},

{507, 79567}, {508, 79760}, {509, 79953}, {510, 80146}, {511, 80339},

{512, 80532}, {513, 80724}, {514, 80917}, {515, 81109}, {516, 81301},

{517, 81492}, {518, 81684}, {519, 81875}, {520, 82066}, {521, 82258},

{522, 82448}, {523, 82639}, {524, 82830}, {525, 83020}, {526, 83210},

{527, 83400}, {528, 83590}, {529, 83779}, {530, 83969}, {531, 84158},

{532, 84347}, {533, 84536}, {534, 84725}, {535, 84913}, {536, 85102},

{537, 85290}, {538, 85478}, {539, 85666}, {540, 85854}, {541, 86041},

{542, 86228}, {543, 86416}, {544, 86603}, {545, 86789}, {546, 86976},

{547, 87163}, {548, 87349}, {549, 87535}, {550, 87721}, {551, 87907},

{552, 88092}, {553, 88278}, {554, 88463}, {555, 88648}, {556, 88833},

{557, 89018}, {558, 89203}, {559, 89387}, {560, 89571}, {561, 89755},

{562, 89939}, {563, 90123}, {564, 90307}, {565, 90490}, {566, 90674},

{567, 90857}, {568, 91040}, {569, 91222}, {570, 91405}, {571, 91587},

{572, 91770}, {573, 91952}, {574, 92134}, {575, 92315}, {576, 92497},

{577, 92679}, {578, 92860}, {579, 93041}, {580, 93222}, {581, 93403},

{582, 93583}, {583, 93764}, {584, 93944}, {585, 94124}, {586, 94304},

{587, 94484}, {588, 94664}, {589, 94843}, {590, 95023}, {591, 95202},

{592, 95381}, {593, 95560}, {594, 95738}, {595, 95917}, {596, 96095},

{597, 96273}, {598, 96451}, {599, 96629}, {600, 96807}, {601, 96985},

{602, 97162}, {603, 97339}, {604, 97516}, {605, 97693}, {606, 97870},

{607, 98047}, {608, 98223}, {609, 98399}, {610, 98576}, {611, 98752},

{612, 98927}, {613, 99103}, {614, 99279}, {615, 99454}, {616, 99629},

{617, 99804}, {618, 99979}, {619, 100154}, {620, 100328}, {621, 100503},

{622, 100677}, {623, 100851}, {624, 101025}, {625, 101199}, {626, 101373},

{627, 101546}, {628, 101720}, {629, 101893}, {630, 102066}, {631, 102239},

{632, 102411}, {633, 102584}, {634, 102756}, {635, 102929}, {636, 103101},

{637, 103273}, {638, 103445}, {639, 103616}, {640, 103788}, {641, 103959},

{642, 104130}, {643, 104302}, {644, 104472}, {645, 104643}, {646, 104814},

{647, 104984}, {648, 105155}, {649, 105325}, {650, 105495}, {651, 105665},

{652, 105835}, {653, 106004}, {654, 106174}, {655, 106343}, {656, 106512},

{657, 106681}, {658, 106850}, {659, 107019}, {660, 107187}, {661, 107355},

{662, 107524}, {663, 107692}, {664, 107860}, {665, 108028}, {666, 108195},

{667, 108363}, {668, 108530}, {669, 108697}, {670, 108865}, {671, 109031},

{672, 109198}, {673, 109365}, {674, 109531}, {675, 109698}, {676, 109864},

{677, 110030}, {678, 110196}, {679, 110362}, {680, 110528}, {681, 110693},

{682, 110858}, {683, 111024}, {684, 111189}, {685, 111354}, {686, 111518},

{687, 111683}, {688, 111848}, {689, 112012}, {690, 112176}, {691, 112340},

{692, 112504}, {693, 112668}, {694, 112832}, {695, 112995}, {696, 113159},

{697, 113322}, {698, 113485}, {699, 113648}, {700, 113811}, {701, 113973},

{702, 114136}, {703, 114298}, {704, 114461}, {705, 114623}, {706, 114785},

{707, 114947}, {708, 115108}, {709, 115270}, {710, 115431}, {711, 115593},

{712, 115754}, {713, 115915}, {714, 116076}, {715, 116236}, {716, 116397},

{717, 116558}, {718, 116718}, {719, 116878}, {720, 117038}, {721, 117198},

{722, 117358}, {723, 117518}, {724, 117677}, {725, 117836}, {726, 117996},

{727, 118155}, {728, 118314}, {729, 118473}, {730, 118631}, {731, 118790},

{732, 118948}, {733, 119107}, {734, 119265}, {735, 119423}, {736, 119581},

{737, 119739}, {738, 119896}, {739, 120054}, {740, 120211}, {741, 120368},

{742, 120525}, {743, 120682}, {744, 120839}, {745, 120996}, {746, 121153},

{747, 121309}, {748, 121465}, {749, 121622}, {750, 121778}, {751, 121934},

{752, 122089}, {753, 122245}, {754, 122400}, {755, 122556}, {756, 122711},

{757, 122866}, {758, 123021}, {759, 123176}, {760, 123331}, {761, 123486},

{762, 123640}, {763, 123794}, {764, 123949}, {765, 124103}, {766, 124257},

{767, 124411}, {768, 124564}, {769, 124718}, {770, 124871}, {771, 125025},
};

enum { NUM_RAW_KMB = ARRAY_SIZE(raw_kmb_data) };


static int TempLookupSearch(int low, int high, int n)
{
	int mid;

	mid = low + (high - low) / 2;
	if (raw_kmb_data[mid].N == n)
		return raw_kmb_data[mid].temp;
	if (raw_kmb_data[mid].N > n)
		return TempLookupSearch(low, mid - 1, n);
	if (raw_kmb_data[mid].N < n)
		return TempLookupSearch(mid + 1, high, n);
	return 0;
}

static int kmb_sensor_read_temp(void __iomem *regs_val,
						int offset,
						int sample_valid_mask,
						int sample_value,
						int bit_shift,
						int *temp)
{
	int result;
	/* clear the bit of TSENS_EN and re-enable again */
	iowrite32(0x00, regs_val+AON_TSENS_CFG);
	iowrite32(CFG_MASK_MANUAL, regs_val+AON_TSENS_CFG);
	*temp = ioread32(regs_val+offset);
	if (*temp & sample_valid_mask) {
		*temp = (*temp >> bit_shift & sample_value);
		if (*temp >= Lower_Temp_Nrange && *temp <= Upper_Temp_Nrange) {
			result = TempLookupSearch(0, NUM_RAW_KMB - 1, *temp);
			*temp = result;
		} else {
			if (*temp < Lower_Temp_Nrange)
				*temp = Lower_Temp;
			else
				*temp = Upper_Temp;
		}
	} else {
		*temp = -255;
	}
	return 0;
}

int kmb_tj_temp_list[6];

int *kmb_tj_get_temp_base(void)
{
	return kmb_tj_temp_list;
}
EXPORT_SYMBOL_GPL(kmb_tj_get_temp_base);

static int keembay_get_temp(struct thermal_zone_device *thermal,
							int *temp)
{
	struct kmb_trip_point_info *kmb_zone_info = thermal->devdata;
	struct keembay_therm_info *ktherm = kmb_zone_info->thermal_info;

	spin_lock(&ktherm->lock);
	switch (kmb_zone_info->sensor_type) {
	case KEEMBAY_SENSOR_MSS:
			kmb_sensor_read_temp(ktherm->regs_val,
					AON_TSENS_DATA0,
					MSS_T_SAMPLE_VALID,
					MSS_T_SAMPLE,
					MSS_BIT_SHIFT,
					temp);
			ktherm->mss = *temp;
			kmb_tj_temp_list[2] = ktherm->mss;
			break;

	case KEEMBAY_SENSOR_CSS:
			kmb_sensor_read_temp(ktherm->regs_val,
					AON_TSENS_DATA0,
					CSS_T_SAMPLE_VALID,
					CSS_T_SAMPLE,
					CSS_BIT_SHIFT,
					temp);
			ktherm->css = *temp;
			kmb_tj_temp_list[3] = ktherm->css;
			break;

	case KEEMBAY_SENSOR_NCE:
			kmb_sensor_read_temp(ktherm->regs_val,
					AON_TSENS_DATA1,
					NCE0_T_SAMPLE_VALID,
					NCE0_T_SAMPLE,
					NCE0_BIT_SHIFT,
					&ktherm->nce0);
			kmb_sensor_read_temp(ktherm->regs_val,
					AON_TSENS_DATA1,
					NCE1_T_SAMPLE_VALID,
					NCE1_T_SAMPLE,
					NCE1_BIT_SHIFT,
					&ktherm->nce1);
			kmb_tj_temp_list[4] = ktherm->nce0;
			kmb_tj_temp_list[5] = ktherm->nce1;
			ktherm->nce = ktherm->nce1;
			*temp = ktherm->nce1;
			if (ktherm->nce0 > ktherm->nce1) {
				ktherm->nce = ktherm->nce0;
				*temp = ktherm->nce0;
			}
			kmb_tj_temp_list[1] = *temp;
			break;

	case KEEMBAY_SENSOR_SOC:
			//temp = css > mss ? (css > nce ? css : nce)
			//: (mss > nce ? mss : nce);
			*temp = ktherm->css > ktherm->mss ?
					(ktherm->css > ktherm->nce ?
					ktherm->css : ktherm->nce)
					: (ktherm->mss > ktherm->css ?
					ktherm->mss : ktherm->css);
					kmb_tj_temp_list[0] = *temp;
			break;
	default:
			break;
	}
	spin_unlock(&ktherm->lock);
	return 0;
}
EXPORT_SYMBOL(kmb_tj_temp_list);

static int keembay_thermal_get_trip_type(struct thermal_zone_device *zone,
			int trip, enum thermal_trip_type *type)
{

	struct kmb_trip_point_info *kmb_zone_info = zone->devdata;

	*type = kmb_zone_info->trip_info[trip].trip_type;
	return 0;
}


static int keembay_thermal_get_trip_temp(struct thermal_zone_device *zone,
				int trip, int *temp)
{

	struct kmb_trip_point_info *kmb_zone_info = zone->devdata;

	*temp = kmb_zone_info->trip_info[trip].temperature;
	return 0;
}

/* Refer https://lwn.net/Articles/242046/ how to receive this event in userspace */
int notify_user_space(struct thermal_zone_device *tz, int trip)
{
	char *thermal_prop[5];
	int i;

	mutex_lock(&tz->lock);
	thermal_prop[0] = kasprintf(GFP_KERNEL, "NAME=%s", tz->type);
	thermal_prop[1] = kasprintf(GFP_KERNEL, "TEMP=%d",
						tz->emul_temperature);
	thermal_prop[2] = kasprintf(GFP_KERNEL, "TRIP=%d", trip);
	thermal_prop[3] = kasprintf(GFP_KERNEL, "EVENT=%d", tz->notify_event);
	thermal_prop[4] = NULL;
	kobject_uevent_env(&tz->device.kobj, KOBJ_CHANGE, thermal_prop);
	for (i = 0; i < 4; ++i)
		kfree(thermal_prop[i]);
	mutex_unlock(&tz->lock);
	return 0;
}


static int keembay_thermal_notify(struct thermal_zone_device *zone,
			       int trip, enum thermal_trip_type type)
{
	struct kmb_trip_point_info *kmb_zone_info = zone->devdata;
	struct keembay_therm_info *ktherm = kmb_zone_info->thermal_info;

	notify_user_space(zone, trip);
	switch (type) {
	case THERMAL_TRIP_PASSIVE:
		dev_warn(ktherm->dev, "Thermal reached to passive temperature\n");
		break;
	case THERMAL_TRIP_CRITICAL:
		dev_warn(ktherm->dev, "Thermal reached to critical temperature\n");
		break;
	default:
		dev_warn(ktherm->dev, "Thermal not reached to passive temperature\n");
		break;
	}
	return 0;
}

static int keembay_bind(struct thermal_zone_device *tz,
		    struct thermal_cooling_device *cdev)
{
	int ret;

	/*Check here thermal device zone name and*/
	/*cdev name to match, then call the bind device */
	if (strncmp(tz->type, cdev->type, THERMAL_NAME_LENGTH) == 0) {
		ret = thermal_zone_bind_cooling_device
				(tz,
				KEEMBAY_TRIP_PASSIVE,
				cdev,
				THERMAL_NO_LIMIT,
				THERMAL_NO_LIMIT,
				THERMAL_WEIGHT_DEFAULT);
		if (ret) {
			dev_err(&tz->device,
				"binding zone %s with cdev %s failed:%d\n",
				tz->type, cdev->type, ret);
			return ret;
		}
	}
	return 0;
}

static int keembay_unbind(struct thermal_zone_device *tz,
		      struct thermal_cooling_device *cdev)
{
	int ret;

	ret = thermal_zone_unbind_cooling_device(tz, 0, cdev);
	if (ret) {
		dev_err(&tz->device,
			"unbinding zone %s with cdev %s failed:%d\n",
			tz->type, cdev->type, ret);
		return ret;
	}
	return 0;
}

static struct thermal_zone_device_ops ops = {
	.bind = keembay_bind,
	.unbind = keembay_unbind,
	.get_temp = keembay_get_temp,
	.get_trip_type	= keembay_thermal_get_trip_type,
	.get_trip_temp	= keembay_thermal_get_trip_temp,
	.notify		= keembay_thermal_notify,
/*	.set_emul_temp = keembay_thermal_emulation */

};


static const struct of_device_id keembay_thermal_id_table[] = {
	{ .compatible = "intel,keembay-tsens" },
	{}
};

struct keembay_therm_info *g_thermal_data;


static int hddl_device_thermal_init(void);
static int hddl_device_thermal_exit(void);

static int keembay_thermal_probe(struct platform_device *pdev)
{
	int ret;
	int error;

	dev_info(&pdev->dev, "Keembay thermal probe\n");

	g_thermal_data = devm_kzalloc(&pdev->dev,
					sizeof(struct keembay_therm_info),
					GFP_KERNEL);
	if (!g_thermal_data)
		return -ENOMEM;
	/* spin lock init */
	spin_lock_init(&g_thermal_data->lock);
	g_thermal_data->regs_val = ioremap(AON_INTERFACE, 32);
	/* getting clk */
	g_thermal_data->thermal_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(g_thermal_data->thermal_clk)) {
		ret = PTR_ERR(g_thermal_data->thermal_clk);
		if (ret != -EPROBE_DEFER) {
			dev_err(&pdev->dev,
					"failed to get thermal clk: %d\n", ret);
		}
		return PTR_ERR(g_thermal_data->thermal_clk);
	}
	ret = clk_prepare_enable(g_thermal_data->thermal_clk);
	if (ret) {
		dev_err(&pdev->dev,
		"failed to enable thermal clk: %d\n",
		ret);
	}
	//Temperature sensor clock must be in the range 1.25MHz.
	ret = clk_set_rate(g_thermal_data->thermal_clk, 1250000);
	ret = clk_prepare_enable(g_thermal_data->thermal_clk);
	error = clk_enable(g_thermal_data->thermal_clk);
	if (error)
		return error;

#if defined(MODULE)
	hddl_device_thermal_init();
#endif /* MODULE */

	return 0;
}

int keembay_thermal_zone_register(struct kmb_trip_point_info *zone_trip_info)
{
	int ret;

	zone_trip_info->thermal_info = g_thermal_data;
	zone_trip_info->tz =  thermal_zone_device_register(
		zone_trip_info->sensor_name,
		zone_trip_info->n_trips,
		0,
		zone_trip_info,
		&ops,
		NULL,
		zone_trip_info->passive_delay,
		zone_trip_info->polling_delay
		);
	if (IS_ERR(zone_trip_info->tz)) {
		ret = PTR_ERR(zone_trip_info->tz);
		dev_err(g_thermal_data->dev,
			"failed to register thermal zone device %d\n", ret);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(keembay_thermal_zone_register);

/* Zone Exit */
int keembay_thermal_zone_unregister(struct kmb_trip_point_info *zone_trip_info)
{
	thermal_zone_device_unregister(zone_trip_info->tz);
	return 0;
}
EXPORT_SYMBOL_GPL(keembay_thermal_zone_unregister);


/* Device Exit */
static int keembay_thermal_exit(struct platform_device *pdev)
{
	struct thermal_zone_device *keembay_thermal =
				platform_get_drvdata(pdev);

#if defined(MODULE)
	hddl_device_thermal_exit();
#endif /* MODULE */

	thermal_zone_device_unregister(keembay_thermal);
	clk_disable_unprepare(g_thermal_data->thermal_clk);

	return 0;
}
MODULE_DEVICE_TABLE(of, keembay_thermal_id_table);

static struct platform_driver keembay_thermal_driver = {
	.probe = keembay_thermal_probe,
	.remove = keembay_thermal_exit,
	.driver = {
		.name = "keembay_thermal",
		.of_match_table = keembay_thermal_id_table,
	},
};

module_platform_driver(keembay_thermal_driver);


struct kmb_trip_point_info mss_zone_trip_info = {
	.sensor_type = KEEMBAY_SENSOR_MSS,
	.sensor_name = "mss",
	.passive_delay = 1000,
	.polling_delay = 2000,
	.n_trips = 3,
	.trip_info = {
		{ THERMAL_TRIP_PASSIVE, 40000 },
		{ THERMAL_TRIP_PASSIVE, 80000 },
		{ THERMAL_TRIP_CRITICAL, 1000000 },
	},
	NULL,
	NULL,
};

struct kmb_trip_point_info css_zone_trip_info = {
	.sensor_type = KEEMBAY_SENSOR_CSS,
	.sensor_name = "css",
	.passive_delay = 1000,
	.polling_delay = 2000,
	.n_trips = 3,
	.trip_info = {
		{ THERMAL_TRIP_PASSIVE, 40000 },
		{ THERMAL_TRIP_PASSIVE, 80000 },
		{ THERMAL_TRIP_CRITICAL, 1000000 },
	},
	NULL,
	NULL,
};

struct kmb_trip_point_info nce_zone_trip_info = {
	.sensor_type = KEEMBAY_SENSOR_NCE,
	.sensor_name = "nce",
	.passive_delay = 1000,
	.polling_delay = 2000,
	.n_trips = 3,
	.trip_info = {
		{ THERMAL_TRIP_PASSIVE, 40000 },
		{ THERMAL_TRIP_PASSIVE, 80000 },
		{ THERMAL_TRIP_CRITICAL, 1000000 },
	},
	NULL,
	NULL,
};

struct kmb_trip_point_info soc_zone_trip_info = {
	.sensor_type = KEEMBAY_SENSOR_SOC,
	.sensor_name = "keembay_thermal",
	.passive_delay = 1000,
	.polling_delay = 2000,
	.n_trips = 2,
	.trip_info = {
		{ THERMAL_TRIP_PASSIVE, 80000 },
		{ THERMAL_TRIP_CRITICAL, 1000000 },
	},
	NULL,
	NULL,
};

static int hddl_device_thermal_init(void)
{
	keembay_thermal_zone_register(&mss_zone_trip_info);
	keembay_thermal_zone_register(&css_zone_trip_info);
	keembay_thermal_zone_register(&nce_zone_trip_info);
	keembay_thermal_zone_register(&soc_zone_trip_info);
	return 0;
};

static int hddl_device_thermal_exit(void)
{
	return 0;
};

#if !defined(MODULE)
late_initcall(hddl_device_thermal_init);
late_initcall(hddl_device_thermal_exit);
#endif

MODULE_DESCRIPTION("KeemBay Thermal Driver");
MODULE_AUTHOR("Sandeep Singh <sandeep1.singh@intel.com>");
MODULE_AUTHOR("Raja Subramanian, Lakshmi Bai <lakshmi.bai.raja.subramanian@intel.com>");
MODULE_LICENSE("GPL v2");

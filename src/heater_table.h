
#ifndef HEATER_TABLE_H
#define HEATER_TABLE_H

#include "heater.h"

const short temptable[NUMTEMPS][2] = {
{	23	,	300	},
{	27	,	290	},
{	28	,	285	},
{	31	,	280	},
{	33	,	275	},
{	35	,	270	},
{	38	,	265	},
{	41	,	260	},
{	44	,	255	},
{	48	,	250	},
{	52	,	245	},
{	56	,	240	},
{	61	,	235	},
{	66	,	230	},
{	71	,	225	},
{	78	,	220	},
{	84	,	215	},
{	92	,	210	},
{	100	,	205	},
{	109	,	200	},
{	120	,	195	},
{	131	,	190	},
{	143	,	185	},
{	156	,	180	},
{	171	,	175	},
{	187	,	170	},
{	205	,	165	},
{	224	,	160	},
{	245	,	155	},
{	268	,	150	},
{	293	,	145	},
{	320	,	140	},
{	348	,	135	},
{	379	,	130	},
{	411	,	125	},
{	445	,	120	},
{	480	,	115	},
{	516	,	110	},
{	553	,	105	},
{	591	,	100	},
{	628	,	95	},
{	665	,	90	},
{	702	,	85	},
{	737	,	80	},
{	770	,	75	},
{	801	,	70	},
{	830	,	65	},
{	857	,	60	},
{	881	,	55	},
{	903	,	50	},
{	922	,	45	},
{	939	,	40	},
{	954	,	35	},
{	966	,	30	},
{	977	,	25	},
{	985	,	20	},
{	993	,	15	},
{	999	,	10	},
{	1008	,	0	}, //safety
{	1023	, 	-40	}
};


// Rough calibration for Makibox A6 Hotbed (initial Beta version).
// Calibrated to temperature near centre of the Hotbed using a K-Type 
// Thermocouple.
// Note: temperature differences / gradients around other areas of the 
// Hotbed have been ignored for now.
const short bedtemptable[BNUMTEMPS][2] = {
{	35	,	180	},		// Extrapolated Value - Hotbed can not reach this temp
{	211	,	140	},
{	233	,	135	},
{	261	,	130	},
{	290	,	125	},
{	328	,	120	},
{	362	,	115	},
{	406	,	110	},
{	446	,	105	},
{	496	,	100	},
{	539	,	95	},
{	585	,	90	},
{	629	,	85	},
{	675	,	80	},
{	718	,	75	},
{	758	,	70	},
{	793	,	65	},
{	822	,	60	},
{	841	,	55	},
{	876	,	50	},
{	899	,	45	},
{	926	,	40	},
{	946	,	35	},
{	962	,	30	},
{	977	,	25	},
{	987	,	20	},		// Assumed Value - Not Calibrated 
{	995	,	15	},		// Assumed Value - Not Calibrated
{	1001	,	10	},		// Assumed Value - Not Calibrated
{	1010	,	0	}, 	//safety // Assumed Value - Not Calibrated
{	1023	,	-40	}
};

#endif
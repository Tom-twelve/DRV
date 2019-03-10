/**
 ******************************************************************************
 * @file		AngleTable.c
 * @author		WrathWings
 * @version 	V1.0
 * @date		2019.1.17
 * @brief		Angle table of motors
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
 
 /* Private includes ----------------------------------------------------------*/
/* CODE BEGIN Includes */
#include "AngleTable.h"
/* CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* CODE BEGIN PTD */
#if RobotIdentifier == 1U
	#if CAN_ID_NUM == 1
	const short int EleAngleRef[] = 
	{144	,
	162	,
	180	,
	198	,
	216	,
	234	,
	252	,
	270	,
	288	,
	306	,
	324	,
	342	,
	360	,
	378	,
	396	,
	414	,
	432	,
	450	,
	468	,
	486	,
	504	,
	522	,
	540	,
	558	,
	576	,
	594	,
	612	,
	630	,
	648	,
	666	,
	684	,
	702	,
	720	,
	738	,
	756	,
	774	,
	792	,
	810	,
	828	,
	846	,
	864	,
	882	,
	900	,
	918	,
	936	,
	954	,
	972	,
	990	,
	1008	,
	1026	,
	1044	,
	1062	,
	1080	,
	1098	,
	1116	,
	1134	,
	1152	,
	1170	,
	1188	,
	1206	,
	1224	,
	1242	,
	1260	,
	1278	,
	1296	,
	1314	,
	1332	,
	1350	,
	1368	,
	1386	,
	1404	,
	1422	,
	1440	,
	1458	,
	1476	,
	1494	,
	1512	,
	1530	,
	1548	,
	1566	,
	1584	,
	1602	,
	1620	,
	1638	,
	1656	,
	1674	,
	1692	,
	1710	,
	1728	,
	1746	,
	1764	,
	1782	,
	1800	,
	1818	,
	1836	,
	1854	,
	1872	,
	1890	,
	1908	,
	1926	,
	1944	,
	1962	,
	1980	,
	1998	,
	2016	,
	2034	,
	2052	,
	2070	,
	2088	,
	2106	,
	2124	,
	2142	,
	2160	,
	2178	,
	2196	,
	2214	,
	2232	,
	2250	,
	2268	,
	2286	,
	2304	,
	2322	,
	2340	,
	2358	,
	2376	,
	2394	,
	2412	,
	2430	,
	2448	,
	2466	,
	2484	,
	2502	,
	2520	,
	2538	,
	2556	,
	2574	,
	2592	,
	2610	,
	2628	,
	2646	,
	2664	,
	2682	
	};

	const int MecAngleRef[] = 
	{-306	,
	49	,
	63	,
	448	,
	813	,
	824	,
	1085	,
	1348	,
	1554	,
	1616	,
	2018	,
	2366	,
	2378	,
	2809	,
	3174	,
	3183	,
	3443	,
	3795	,
	3929	,
	3970	,
	4390	,
	4698	,
	4710	,
	5103	,
	5531	,
	5543	,
	5802	,
	6175	,
	6271	,
	6330	,
	6758	,
	7074	,
	7444	,
	7445	,
	7829	,
	7842	,
	8136	,
	8501	,
	8628	,
	8672	,
	9076	,
	9405	,
	9416	,
	9792	,
	10162	,
	10172	,
	10272	,
	10858	,
	10964	,
	11027	,
	11484	,
	11770	,
	11782	,
	12177	,
	12544	,
	12555	,
	12743	,
	13047	,
	13293	,
	13345	,
	13805	,
	14131	,
	14142	,
	14510	,
	14893	,
	14904	,
	15026	,
	15445	,
	15604	,
	15659	,
	16127	,
	16468	,
	16479	,
	16875	,
	17221	,
	17231	,
	17496	,
	17766	,
	17980	,
	18027	,
	18417	,
	18768	,
	18783	,
	19207	,
	19588	,
	19599	,
	19853	,
	20200	,
	20330	,
	20385	,
	20818	,
	21111	,
	21122	,
	21522	,
	21929	,
	21940	,
	22200	,
	22568	,
	22675	,
	22719	,
	23135	,
	23453	,
	23466	,
	23814	,
	24212	,
	24226	,
	24521	,
	24878	,
	24989	,
	25049	,
	25466	,
	25778	,
	25788	,
	26169	,
	26515	,
	26527	,
	26634	,
	27221	,
	27335	,
	27382	,
	27811	,
	28117	,
	28131	,
	28521	,
	28900	,
	28912	,
	29080	,
	29396	,
	29634	,
	29706	,
	30185	,
	30492	,
	30503	,
	30879	,
	31245	,
	31256	,
	31390	,
	31797	,
	31968	,
	32008	,
	32462	,
	32817	
	};
	#elif CAN_ID_NUM == 2
	const short int EleAngleRef[] = 
	{144	,
	162	,
	180	,
	198	,
	216	,
	234	,
	252	,
	270	,
	288	,
	306	,
	324	,
	342	,
	360	,
	378	,
	396	,
	414	,
	432	,
	450	,
	468	,
	486	,
	504	,
	522	,
	540	,
	558	,
	576	,
	594	,
	612	,
	630	,
	648	,
	666	,
	684	,
	702	,
	720	,
	738	,
	756	,
	774	,
	792	,
	810	,
	828	,
	846	,
	864	,
	882	,
	900	,
	918	,
	936	,
	954	,
	972	,
	990	,
	1008	,
	1026	,
	1044	,
	1062	,
	1080	,
	1098	,
	1116	,
	1134	,
	1152	,
	1170	,
	1188	,
	1206	,
	1224	,
	1242	,
	1260	,
	1278	,
	1296	,
	1314	,
	1332	,
	1350	,
	1368	,
	1386	,
	1404	,
	1422	,
	1440	,
	1458	,
	1476	,
	1494	,
	1512	,
	1530	,
	1548	,
	1566	,
	1584	,
	1602	,
	1620	,
	1638	,
	1656	,
	1674	,
	1692	,
	1710	,
	1728	,
	1746	,
	1764	,
	1782	,
	1800	,
	1818	,
	1836	,
	1854	,
	1872	,
	1890	,
	1908	,
	1926	,
	1944	,
	1962	,
	1980	,
	1998	,
	2016	,
	2034	,
	2052	,
	2070	,
	2088	,
	2106	,
	2124	,
	2142	,
	2160	,
	2178	,
	2196	,
	2214	,
	2232	,
	2250	,
	2268	,
	2286	,
	2304	,
	2322	,
	2340	,
	2358	,
	2376	,
	2394	,
	2412	,
	2430	,
	2448	,
	2466	,
	2484	,
	2502	,
	2520	,
	2538	,
	2556	,
	2574	,
	2592	,
	2610	,
	2628	,
	2646	,
	2664	,
	2682	
	};

	const int MecAngleRef[] = 
	{-306	,
	49	,
	63	,
	448	,
	813	,
	824	,
	1085	,
	1348	,
	1554	,
	1616	,
	2018	,
	2366	,
	2378	,
	2809	,
	3174	,
	3183	,
	3443	,
	3795	,
	3929	,
	3970	,
	4390	,
	4698	,
	4710	,
	5103	,
	5531	,
	5543	,
	5802	,
	6175	,
	6271	,
	6330	,
	6758	,
	7074	,
	7444	,
	7445	,
	7829	,
	7842	,
	8136	,
	8501	,
	8628	,
	8672	,
	9076	,
	9405	,
	9416	,
	9792	,
	10162	,
	10172	,
	10272	,
	10858	,
	10964	,
	11027	,
	11484	,
	11770	,
	11782	,
	12177	,
	12544	,
	12555	,
	12743	,
	13047	,
	13293	,
	13345	,
	13805	,
	14131	,
	14142	,
	14510	,
	14893	,
	14904	,
	15026	,
	15445	,
	15604	,
	15659	,
	16127	,
	16468	,
	16479	,
	16875	,
	17221	,
	17231	,
	17496	,
	17766	,
	17980	,
	18027	,
	18417	,
	18768	,
	18783	,
	19207	,
	19588	,
	19599	,
	19853	,
	20200	,
	20330	,
	20385	,
	20818	,
	21111	,
	21122	,
	21522	,
	21929	,
	21940	,
	22200	,
	22568	,
	22675	,
	22719	,
	23135	,
	23453	,
	23466	,
	23814	,
	24212	,
	24226	,
	24521	,
	24878	,
	24989	,
	25049	,
	25466	,
	25778	,
	25788	,
	26169	,
	26515	,
	26527	,
	26634	,
	27221	,
	27335	,
	27382	,
	27811	,
	28117	,
	28131	,
	28521	,
	28900	,
	28912	,
	29080	,
	29396	,
	29634	,
	29706	,
	30185	,
	30492	,
	30503	,
	30879	,
	31245	,
	31256	,
	31390	,
	31797	,
	31968	,
	32008	,
	32462	,
	32817	
	};
	#elif CAN_ID_NUM == 3
	const short int EleAngleRef[] = 
	{18	,
	36	,
	54	,
	72	,
	90	,
	108	,
	126	,
	144	,
	162	,
	180	,
	198	,
	216	,
	234	,
	252	,
	270	,
	288	,
	306	,
	324	,
	342	,
	360	,
	378	,
	396	,
	414	,
	432	,
	450	,
	468	,
	486	,
	504	,
	522	,
	540	,
	558	,
	576	,
	594	,
	612	,
	630	,
	648	,
	666	,
	684	,
	702	,
	720	,
	738	,
	756	,
	774	,
	792	,
	810	,
	828	,
	846	,
	864	,
	882	,
	900	,
	918	,
	936	,
	954	,
	972	,
	990	,
	1008	,
	1026	,
	1044	,
	1062	,
	1080	,
	1098	,
	1116	,
	1134	,
	1152	,
	1170	,
	1188	,
	1206	,
	1224	,
	1242	,
	1260	,
	1278	,
	1296	,
	1314	,
	1332	,
	1350	,
	1368	,
	1386	,
	1404	,
	1422	,
	1440	,
	1458	,
	1476	,
	1494	,
	1512	,
	1530	,
	1548	,
	1566	,
	1584	,
	1602	,
	1620	,
	1638	,
	1656	,
	1674	,
	1692	,
	1710	,
	1728	,
	1746	,
	1764	,
	1782	,
	1800	,
	1818	,
	1836	,
	1854	,
	1872	,
	1890	,
	1908	,
	1926	,
	1944	,
	1962	,
	1980	,
	1998	,
	2016	,
	2034	,
	2052	,
	2070	,
	2088	,
	2106	,
	2124	,
	2142	,
	2160	,
	2178	,
	2196	,
	2214	,
	2232	,
	2250	,
	2268	,
	2286	,
	2304	,
	2322	,
	2340	,
	2358	,
	2376	,
	2394	,
	2412	,
	2430	,
	2448	,
	2466	,
	2484	,
	2502	,
	2520	,
	2538	,
	2556	,
	2574	,
	2592	,
	2610	,
	2628	,
	2646	,
	2664	,
	2682	,
	2700	,
	2718	,
	2736	,
	2754	,
	2772	,
	2790	,
	2808	,
	2826	,
	2844	,
	2862	,
	2880	,
	2898	,
	2916	,
	2934	,
	2952	,
	2970	,
	2988	,
	3006	,
	3024	,
	3042	,
	3060	,
	3078	,
	3096	,
	3114	,
	3132	,
	3150	,
	3168	,
	3186	,
	3204	,
	3222	,
	3240	,
	3258	,
	3276	,
	3294	,
	3312	,
	3330	,
	3348	,
	3366	,
	3384	,
	3402	,
	3420	,
	3438	,
	3456	,
	3474	,
	3492	,
	3510	,
	3528	,
	3546	,
	3564	,
	3582	,
	3600	,
	3618	,
	3636	,
	3654	,
	3672	,
	3690	,
	3708	,
	3726	,
	3744	,
	3762	,
	3780	,
	3798	,
	3816	,
	3834	,
	3852	,
	3870	,
	3888	,
	3906	,
	3924	,
	3942	,
	3960	,
	3978	,
	3996	,
	4014	,
	4032	,
	4050	,
	4068	,
	4086	,
	4104	,
	4122	,
	4140	,
	4158	,
	4176	,
	4194	,
	4212	,
	4230	,
	4248	,
	4266	,
	4284	,
	4302	,
	4320	,
	4338	,
	4356	,
	4374	,
	4392	,
	4410	,
	4428	,
	4446	,
	4464	,
	4482	,
	4500	,
	4518	,
	4536	,
	4554	,
	4572	,
	4590	,
	4608	,
	4626	,
	4644	,
	4662	,
	4680	,
	4698	,
	4716	,
	4734	,
	4752	,
	4770	,
	4788	,
	4806	,
	4824	,
	4842	,
	4860	,
	4878	,
	4896	,
	4914	,
	4932	,
	4950	,
	4968	,
	4986	,
	5004	,
	5022	,
	5040	,
	5058	,
	5076	
	};

	const int MecAngleRef[] = 
	{-60	,
	209	,
	217	,
	337	,
	554	,
	607	,
	636	,
	861	,
	995	,
	1005	,
	1187	,
	1357	,
	1371	,
	1475	,
	1707	,
	1766	,
	1811	,
	2002	,
	2157	,
	2166	,
	2245	,
	2535	,
	2543	,
	2657	,
	2871	,
	2944	,
	2980	,
	3216	,
	3333	,
	3342	,
	3524	,
	3710	,
	3725	,
	3841	,
	4070	,
	4127	,
	4172	,
	4388	,
	4529	,
	4537	,
	4648	,
	4902	,
	4909	,
	5026	,
	5260	,
	5329	,
	5364	,
	5587	,
	5714	,
	5726	,
	5905	,
	6088	,
	6101	,
	6175	,
	6440	,
	6503	,
	6547	,
	6762	,
	6897	,
	6905	,
	6990	,
	7267	,
	7274	,
	7410	,
	7580	,
	7675	,
	7710	,
	7932	,
	8068	,
	8079	,
	8246	,
	8425	,
	8440	,
	8547	,
	8783	,
	8829	,
	8866	,
	9082	,
	9226	,
	9235	,
	9332	,
	9593	,
	9601	,
	9719	,
	9931	,
	10002	,
	10042	,
	10243	,
	10392	,
	10403	,
	10581	,
	10770	,
	10783	,
	10884	,
	11117	,
	11177	,
	11225	,
	11443	,
	11573	,
	11579	,
	11631	,
	11951	,
	11959	,
	12089	,
	12293	,
	12366	,
	12405	,
	12627	,
	12757	,
	12768	,
	12944	,
	13124	,
	13141	,
	13240	,
	13488	,
	13546	,
	13591	,
	13798	,
	13933	,
	13941	,
	14036	,
	14304	,
	14310	,
	14393	,
	14632	,
	14709	,
	14745	,
	14970	,
	15094	,
	15104	,
	15276	,
	15453	,
	15468	,
	15589	,
	15784	,
	15857	,
	15899	,
	16106	,
	16252	,
	16259	,
	16339	,
	16606	,
	16613	,
	16731	,
	16947	,
	17003	,
	17031	,
	17258	,
	17391	,
	17402	,
	17581	,
	17751	,
	17767	,
	17874	,
	18099	,
	18160	,
	18207	,
	18398	,
	18555	,
	18562	,
	18649	,
	18931	,
	18938	,
	19053	,
	19263	,
	19337	,
	19373	,
	19610	,
	19725	,
	19735	,
	19918	,
	20100	,
	20116	,
	20236	,
	20461	,
	20517	,
	20562	,
	20779	,
	20919	,
	21035	,
	21036	,
	21289	,
	21298	,
	21418	,
	21645	,
	21713	,
	21749	,
	21971	,
	22098	,
	22111	,
	22288	,
	22470	,
	22482	,
	22560	,
	22820	,
	22882	,
	22925	,
	23140	,
	23275	,
	23283	,
	23371	,
	23642	,
	23649	,
	23787	,
	23961	,
	24048	,
	24082	,
	24306	,
	24439	,
	24450	,
	24618	,
	24796	,
	24810	,
	24922	,
	25153	,
	25198	,
	25233	,
	25450	,
	25593	,
	25602	,
	25697	,
	25957	,
	25964	,
	26090	,
	26299	,
	26368	,
	26406	,
	26611	,
	26759	,
	26770	,
	26946	,
	27134	,
	27147	,
	27250	,
	27482	,
	27542	,
	27589	,
	27805	,
	27937	,
	27945	,
	27998	,
	28313	,
	28321	,
	28454	,
	28663	,
	28730	,
	28766	,
	28997	,
	29125	,
	29135	,
	29312	,
	29487	,
	29508	,
	29603	,
	29858	,
	29914	,
	29956	,
	30164	,
	30303	,
	30311	,
	30404	,
	30670	,
	30679	,
	30762	,
	31006	,
	31079	,
	31114	,
	31338	,
	31466	,
	31475	,
	31650	,
	31824	,
	31840	,
	31962	,
	32155	,
	32227	,
	32269	,
	32477	,
	32623	,
	32631	,
	32708	,
	32977	
	};
	#endif
#endif
/* CODE END PTD */

/* Private macro -------------------------------------------------------------*/
/* CODE BEGIN PM */

/* CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* CODE BEGIN PV */

/* CODE END PV */

/* USER CODE BEGIN */

/* USER CODE END */

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/

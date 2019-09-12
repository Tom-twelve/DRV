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
#if POSITION_SENSOR_TYPE == ENCODER_TLE5012
	#if ENCODER_MODE == ENCODER_ABSOLUTE_MODE
		#if ROBOT_ID == 1U
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
			{-80	,
217	,
234	,
591	,
981	,
1000	,
1321	,
1610	,
1734	,
1817	,
2277	,
2567	,
2580	,
2994	,
3300	,
3311	,
3573	,
3893	,
4094	,
4130	,
4573	,
4901	,
4918	,
5330	,
5702	,
5716	,
5956	,
6197	,
6357	,
6468	,
6951	,
7238	,
8707	,
7687	,
8038	,
8050	,
8321	,
8558	,
8705	,
8736	,
9214	,
9570	,
9590	,
9989	,
10395	,
10407	,
10665	,
10960	,
11099	,
11178	,
11547	,
11886	,
11908	,
12383	,
12729	,
12741	,
13042	,
13291	,
13460	,
13500	,
13943	,
14203	,
14219	,
14644	,
15061	,
15074	,
15321	,
15633	,
15776	,
15857	,
16272	,
16565	,
16583	,
16948	,
17329	,
17345	,
17674	,
17930	,
18097	,
18139	,
18624	,
18918	,
18933	,
19348	,
19664	,
19676	,
19941	,
20308	,
20455	,
20534	,
20948	,
21284	,
21300	,
21714	,
22079	,
22091	,
22342	,
22555	,
22757	,
22808	,
23331	,
23619	,
23640	,
24086	,
24441	,
24456	,
24725	,
24992	,
25101	,
25165	,
25624	,
25971	,
25983	,
26389	,
26780	,
26791	,
27053	,
27319	,
27501	,
27537	,
27935	,
28258	,
28288	,
28767	,
29126	,
29140	,
29431	,
29722	,
29856	,
29938	,
30353	,
30603	,
30622	,
31056	,
31462	,
31475	,
31729	,
32004	,
32200	,
32235	,
32688	,
32985	
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
			{-88	,
			222	,
			226	,
			603	,
			1015	,
			1017	,
			1344	,
			1601	,
			1698	,
			1769	,
			2276	,
			2584	,
			2585	,
			3025	,
			3312	,
			3311	,
			3626	,
			3900	,
			4071	,
			4076	,
			4576	,
			4910	,
			4915	,
			5368	,
			5723	,
			5725	,
			5975	,
			6184	,
			6284	,
			6392	,
			6946	,
			7249	,
			7252	,
			7740	,
			8076	,
			8078	,
			8355	,
			8556	,
			8677	,
			8679	,
			9245	,
			9587	,
			9588	,
			10032	,
			10413	,
			10413	,
			10677	,
			10933	,
			11029	,
			11115	,
			11527	,
			11892	,
			11893	,
			12416	,
			12748	,
			12749	,
			13053	,
			13268	,
			13413	,
			13415	,
			13917	,
			14193	,
			14196	,
			14674	,
			15071	,
			15070	,
			15331	,
			15588	,
			15714	,
			15783	,
			16272	,
			16577	,
			16921	,
			16921	,
			17366	,
			17366	,
			17700	,
			17908	,
			18061	,
			18063	,
			18631	,
			18933	,
			18933	,
			19380	,
			19681	,
			19681	,
			19955	,
			20300	,
			20398	,
			20491	,
			20959	,
			21291	,
			21293	,
			21753	,
			22093	,
			22095	,
			22364	,
			22560	,
			22704	,
			22709	,
			23314	,
			23624	,
			23627	,
			24108	,
			24458	,
			24461	,
			24744	,
			24979	,
			25058	,
			25113	,
			25638	,
			25974	,
			25975	,
			26434	,
			26795	,
			26796	,
			27078	,
			27317	,
			27466	,
			27471	,
			27919	,
			28291	,
			28292	,
			28802	,
			29153	,
			29155	,
			29458	,
			29708	,
			29800	,
			29880	,
			30344	,
			30613	,
			30615	,
			31115	,
			31479	,
			31480	,
			31747	,
			32000	,
			32171	,
			32178	,
			32680	,
			32990	
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
			{-1	,
			127	,
			191	,
			355	,
			482	,
			566	,
			650	,
			853	,
			964	,
			978	,
			1152	,
			1277	,
			1317	,
			1503	,
			1656	,
			1729	,
			1838	,
			1980	,
			2102	,
			2142	,
			2313	,
			2448	,
			2517	,
			2680	,
			2804	,
			2896	,
			2994	,
			3197	,
			3297	,
			3314	,
			3495	,
			3621	,
			3668	,
			3857	,
			4013	,
			4086	,
			4195	,
			4352	,
			4466	,
			4509	,
			4681	,
			4805	,
			4887	,
			5052	,
			5178	,
			5277	,
			5369	,
			5571	,
			5673	,
			5689	,
			5870	,
			5995	,
			6038	,
			6221	,
			6383	,
			6457	,
			6564	,
			6719	,
			6831	,
			6874	,
			7046	,
			7171	,
			7244	,
			7406	,
			7518	,
			7622	,
			7712	,
			7915	,
			8019	,
			8035	,
			8210	,
			8328	,
			8373	,
			8558	,
			8716	,
			8783	,
			8882	,
			9046	,
			9157	,
			9202	,
			9378	,
			9494	,
			9571	,
			9733	,
			9852	,
			9944	,
			10045	,
			10235	,
			10342	,
			10361	,
			10542	,
			10659	,
			10715	,
			10894	,
			11052	,
			11124	,
			11237	,
			11389	,
			11506	,
			11543	,
			11728	,
			11845	,
			11928	,
			12091	,
			12215	,
			12304	,
			12406	,
			12606	,
			12702	,
			12725	,
			12905	,
			13012	,
			13072	,
			13260	,
			13416	,
			13494	,
			13599	,
			13754	,
			13862	,
			13909	,
			14085	,
			14203	,
			14281	,
			14434	,
			14562	,
			14649	,
			14749	,
			14948	,
			15041	,
			15064	,
			15241	,
			15347	,
			15402	,
			15584	,
			15724	,
			15813	,
			15910	,
			16067	,
			16179	,
			16381	,
			16383	,
			16533	,
			16584	,
			16743	,
			16883	,
			16958	,
			17049	,
			17231	,
			17351	,
			17378	,
			17537	,
			17673	,
			17713	,
			17884	,
			18044	,
			18124	,
			18230	,
			18373	,
			18502	,
			18536	,
			18701	,
			18851	,
			18906	,
			19066	,
			19203	,
			19286	,
			19386	,
			19574	,
			19687	,
			19710	,
			19877	,
			20018	,
			20057	,
			20240	,
			20402	,
			20480	,
			20584	,
			20741	,
			20863	,
			20895	,
			21064	,
			21209	,
			21266	,
			21437	,
			21573	,
			21660	,
			21756	,
			21943	,
			22060	,
			22080	,
			22244	,
			22388	,
			22420	,
			22598	,
			22765	,
			22842	,
			22944	,
			23100	,
			23218	,
			23251	,
			23417	,
			23558	,
			23613	,
			23781	,
			23904	,
			23996	,
			24088	,
			24279	,
			24396	,
			24415	,
			24575	,
			24710	,
			24746	,
			24924	,
			25090	,
			25158	,
			25254	,
			25414	,
			25534	,
			25568	,
			25738	,
			25874	,
			25931	,
			26101	,
			26230	,
			26315	,
			26417	,
			26598	,
			26717	,
			26735	,
			26902	,
			27043	,
			27082	,
			27262	,
			27426	,
			27499	,
			27604	,
			27762	,
			27882	,
			27912	,
			28090	,
			28228	,
			28290	,
			28463	,
			28591	,
			28679	,
			28776	,
			28973	,
			29084	,
			29102	,
			29269	,
			29400	,
			29445	,
			29632	,
			29798	,
			29872	,
			29975	,
			30129	,
			30247	,
			30281	,
			30452	,
			30591	,
			30647	,
			30815	,
			30942	,
			31032	,
			31126	,
			31321	,
			31430	,
			31445	,
			31615	,
			31742	,
			31782	,
			31965	,
			32112	,
			32193	,
			32290	,
			32448	,
			32566	,
			32603	,
			32767	,
			32895	
			};
			#endif
		#endif
	#elif ENCODER_MODE == ENCODER_INCREMENTAL_MODE
		#if ROBOT_ID == 1U
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
			{0	,
			18	,
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
			5058	
			};

			const int MecAngleRef[] = 
			{-18	,
			66	,
			141	,
			164	,
			231	,
			312	,
			351	,
			382	,
			475	,
			540	,
			553	,
			641	,
			713	,
			735	,
			805	,
			885	,
			924	,
			962	,
			1042	,
			1110	,
			1127	,
			1215	,
			1292	,
			1318	,
			1385	,
			1467	,
			1522	,
			1555	,
			1648	,
			1712	,
			1724	,
			1814	,
			1891	,
			1914	,
			1987	,
			2069	,
			2112	,
			2148	,
			2235	,
			2300	,
			2323	,
			2411	,
			2486	,
			2513	,
			2583	,
			2666	,
			2723	,
			2757	,
			2849	,
			2912	,
			2930	,
			3019	,
			3096	,
			3115	,
			3184	,
			3269	,
			3322	,
			3358	,
			3445	,
			3505	,
			3514	,
			3601	,
			3677	,
			3702	,
			3771	,
			3846	,
			3896	,
			3929	,
			4021	,
			4092	,
			4101	,
			4185	,
			4260	,
			4282	,
			4351	,
			4435	,
			4477	,
			4511	,
			4601	,
			4674	,
			4703	,
			4703	,
			4785	,
			4810	,
			4878	,
			4956	,
			4992	,
			5029	,
			5113	,
			5187	,
			5195	,
			5283	,
			5361	,
			5384	,
			5453	,
			5536	,
			5592	,
			5628	,
			5715	,
			5784	,
			5791	,
			5881	,
			5960	,
			5986	,
			6056	,
			6136	,
			6184	,
			6218	,
			6308	,
			6369	,
			6385	,
			6471	,
			6545	,
			6570	,
			6640	,
			6724	,
			6769	,
			6804	,
			6890	,
			6954	,
			6970	,
			7058	,
			7135	,
			7156	,
			7224	,
			7306	,
			7361	,
			7395	,
			7485	,
			7553	,
			7562	,
			7647	,
			7720	,
			7743	,
			7813	,
			7888	,
			7955	,
			7989	,
			8075	,
			8145	,
			8153	,
			8237	,
			8312	,
			8335	,
			8402	,
			8483	,
			8530	,
			8561	,
			8654	,
			8714	,
			8728	,
			8815	,
			8888	,
			8910	,
			8980	,
			9060	,
			9103	,
			9141	,
			9222	,
			9289	,
			9299	,
			9387	,
			9465	,
			9491	,
			9557	,
			9638	,
			9692	,
			9726	,
			9818	,
			9878	,
			9891	,
			9981	,
			10057	,
			10080	,
			10153	,
			10235	,
			10287	,
			10324	,
			10410	,
			10476	,
			10490	,
			10578	,
			10653	,
			10680	,
			10748	,
			10831	,
			10879	,
			10912	,
			11004	,
			11063	,
			11082	,
			11169	,
			11245	,
			11265	,
			11335	,
			11420	,
			11471	,
			11507	,
			11594	,
			11662	,
			11671	,
			11757	,
			11832	,
			11857	,
			11926	,
			12001	,
			12050	,
			12082	,
			12174	,
			12245	,
			12254	,
			12338	,
			12412	,
			12433	,
			12503	,
			12586	,
			12629	,
			12661	,
			12751	,
			12811	,
			12825	,
			12913	,
			12986	,
			13011	,
			13081	,
			13160	,
			13203	,
			13239	,
			13325	,
			13398	,
			13407	,
			13494	,
			13571	,
			13595	,
			13663	,
			13746	,
			13793	,
			13829	,
			13917	,
			13981	,
			13997	,
			14086	,
			14164	,
			14189	,
			14259	,
			14341	,
			14396	,
			14430	,
			14523	,
			14591	,
			14606	,
			14693	,
			14766	,
			14791	,
			14862	,
			14946	,
			14990	,
			15025	,
			15111	,
			15170	,
			15184	,
			15271	,
			15348	,
			15369	,
			15437	,
			15519	,
			15579	,
			15612	,
			15703	,
			15767	,
			15780	,
			15866	,
			15938	,
			15961	,
			16031	,
			16107	,
			16157	,
			16189	,
			16276	,
			16346	,
			16366	,
			16450	
			};
			#endif
		#endif
	#else
	#error "Encoder Mode Invalid"
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

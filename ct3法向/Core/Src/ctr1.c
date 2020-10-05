#include "ctr.h"

//状态量，当前是否稳定 0 不稳定 1 临界稳定 2 稳定
extern int stableStaus;
//误差量
extern int error;
//上次的误差量
//0 2
extern int lastError;
//积分输出
//0 2
extern int zInt;
//变速积分输出
//2 3
extern int zSlo;
//暂态输出
//0 2
extern int zTrn;
//输出
extern int z;
//补偿输出
//0 1 2 3
extern int zBas;
//消抖稳定状态
//0 2 3
extern int stableCnt;
//消抖稳定方向
//0 1 2 3
extern int stableVec;
//控制模式 0 位置控制 1 速度控制 2 力位控制 3 力速控制
extern uint8_t controlMode;

//积分目标值域
//1
extern int targetP[128];
//当前位置
//1
extern int pt;

//目标值
extern float target;

extern int hPosi;

extern int hForc;

int lpCnt;

//速度减速比
#define SLB 8.0
//位置一阶误差阈值
#define sMaxThre 1000000.0
//位置一阶输出
#define sMaxOp 5000000
//位置二阶误差阈值
#define sSndThre 100000.0
//位置二阶P
#define sSndP 3.0
//位置二阶I
#define sSndI 1.0
//位置三阶I
#define sTrdI 0.5
//位置三阶D
#define sTrdD 1.0


void controlCore()
{
    switch (controlMode)
    {
    case 0: //s
        error = hPosi - target;
        break;
    case 1: //ds
        break;
    case 2: //f
        error = hForc - target;
        break;
    case 3: //df
        break;
    default:
        break;
    }
    //未稳定状态控制
    if (stableStaus == 0)
    {
        z = 0;
    }
    //临界稳定状态控制
    else if (stableStaus == 1)
    {
        switch (controlMode)
        {
        case 0: //s

            break;
        case 1: //ds
            stableVec = 0;
            zBas = 0;
						targetP[0]=hPosi;
            for (int i = 1; i < 128; i++)
            {		
								
                targetP[i] = targetP[i-1] + (int)(target/SLB);
            }
						lpCnt=1;
            break;
        case 2: //f
            break;
        case 3: //df
            stableCnt = 0;
            stableVec = 0;
            zSlo = 0;
            for (int i = 0; i < 128; i++)
            {
                targetP[i] = hForc + (int)((i + 1.0) * target);
            }

            break;

        default:
						z=0;
            break;
        }
    }
    else
    {
        switch (controlMode)
        {
        case 0: //s
            if (error > sMaxThre)
            {
                z = sMaxOp;
                break;
            }
            if (error < -sMaxThre)
            {
                z = -sMaxOp;
                break;
            }
            if (error > sSndThre || error < -sSndThre)
            {
                z = sSndP * error;
                zInt = sSndI * z;
                zBas = 0;
                stableCnt = 0;
                break;
            }

            zInt += error * sTrdI;
            zTrn = (error - lastError)*sTrdD;
            if (zInt > 200000)
                zInt = 200000;
            if (zInt < -200000)
                zInt = -200000;
            z = zInt + zTrn;
            if (error < 10000 && error > -10000)
            {
                //stage 1 wait stablize
                if (stableCnt < 256)
                {
                    stableCnt++;
                }
                //stage 2 collect z
                else if (stableCnt < 512)
                {
                    zBas += z;
                    stableCnt++;
                    stableVec = 0;
                }
                //stage 3 check z
                else if (stableCnt < 768)
                {
                    z = zBas / (256);
                    z = z + error / 2;
                    stableVec += error;
                    stableCnt++;
                }
                //stage 4 stablized
                else
                {
                    if (stableCnt < 1100)
                        stableCnt++;
                    if (stableVec > 0)
                    {
                        if (error > 0)
                        {
                            zInt += error / 2;
                            zTrn = (error - lastError);
                            if (zInt > 200000)
                                zInt = 200000;
                            if (zInt < -200000)
                                zInt = -200000;
                            z = zInt + zTrn;
                        }
                        else
                        {
                            z = zBas / (256);
                            z = z + error / 2;
                        }
                    }
                    else
                    {
                        if (error < 0)
                        {
                            zInt += error / 2;
                            zTrn = (error - lastError);
                            if (zInt > 200000)
                                zInt = 200000;
                            if (zInt < -200000)
                                zInt = -200000;
                            z = zInt + zTrn;
                        }
                        else
                        {
                            z = zBas / (256);
                            z = z + error / 2;
                        }
                    }
                }
            }
            else
            {
                if (stableCnt > 1024)
                {
                    zBas = 0;
                    stableCnt = 0;
                }
            }
            break;
        case 1: //ds
            if (hPosi > targetP[pt])
            {
                z = 200000 + zBas;
                stableVec++;
            }
            else if (hPosi == targetP[pt])
            {
                z = 0;
                if (target > 0)
                    stableVec--;
                if (target < 0)
                    stableVec++;
            }
            else
            {
                z = -200000 + zBas;
                stableVec--;
            }
            if (stableVec > 10)
            {
                zBas += 50000;
                stableVec = 0;
            }
            if (stableVec < -10)
            {
                zBas -= 50000;
                stableVec = 0;
            }
            if (zBas > 5000000)
                zBas = 5000000;
            if (zBas < -5000000)
                zBas = -5000000;
            pt++;
            if (pt == 128)
            {
                pt = 0;
								int dif=hPosi-targetP[127];
								dif=dif<0?-dif:dif;
								int tf=target*10;
								tf=tf<0?-tf:tf;
								lpCnt++;
								if(lpCnt>SLB)lpCnt=SLB;
								if(dif<tf){
										for (int i = 0; i < 128; i++)
										{
												targetP[i] = hPosi + (int)((i + 1.0) * target*(lpCnt/SLB));
										}
								}else{								
										for (int i = 0; i < 128; i++)
										{
												targetP[i] = targetP[127] + (int)((float)(i+1) * target*(lpCnt/SLB));
										}
										
								}
								
            }
            break;
        case 2: //f
            if (error > 1000000)
            {
                z = 1000000;
                break;
            }
            if (error < -1000000)
            {
                z = -1000000;
                break;
            }
            if (error > 100000 || error < -100000)
            {
                z = 3 * error;
                zInt = z;
                zBas = 0;
                stableCnt = 0;
                break;
            }

            zInt += 3 * error;
            zTrn = (error - lastError);
            if (zInt > 200000)
                zInt = 200000;
            if (zInt < -200000)
                zInt = -200000;
            z = zInt + zTrn;
            if (error < 20000 && error > -20000)
            {
                //stage 1 wait stablize
                if (stableCnt < 256)
                {
                    stableCnt++;
                    //stage 2 collect z
                }
                else if (stableCnt < 512)
                {
                    zBas += z;
                    stableCnt++;
                    stableVec = 0;
                    //stage 3 check z
                }
                else if (stableCnt < 768)
                {
                    z = zBas / (256);
                    z = z + 3 * error;
                    stableVec += error;
                    stableCnt++;
                    zSlo = 0;
                    //stage 4 stablized
                }
                else
                {
                    if (stableCnt < 5000)
                        stableCnt++;
                    zSlo += error / 20;
                    if (stableVec > 0)
                    {
                        if (error > 3000)
                        {
                            zInt += 3 * error;
                            zTrn = (error - lastError);
                            if (zInt > 200000)
                                zInt = 200000;
                            if (zInt < -200000)
                                zInt = -200000;
                            z = zInt + zTrn + zSlo;
                        }
                        else
                        {
                            z = zBas / (256);
                            z = z + 3 * error + zSlo;
                        }
                    }
                    else
                    {
                        if (error < -3000)
                        {
                            zInt += 3 * error;
                            zTrn = (error - lastError);
                            if (zInt > 200000)
                                zInt = 200000;
                            if (zInt < -200000)
                                zInt = -200000;
                            z = zInt + zTrn + zSlo;
                        }
                        else
                        {
                            z = zBas / (256);
                            z = z + 3 * error + zSlo;
                        }
                    }
                }
            }
            else
            {
                if (stableCnt > 1024)
                {
                    zBas = 0;
                    stableCnt = 0;
                }
            }
            break;
        case 3: //df
            if (hForc > targetP[pt])
            {
                if (target < 0)
                    z = 50000 + zBas;
                else
                    z = 0;
                stableVec++;
            }
            else if (hForc == targetP[pt])
            {
                z = 0;
                if (target > 0)
                    stableVec--;
                if (target < 0)
                    stableVec++;
            }
            else
            {
                if (target > 0)
                    z = -50000 + zBas;
                else
                    z = 0;
                stableVec--;
            }
            if (stableVec > 10)
            {
                zBas += 10000;
                stableVec = 0;
            }
            if (stableVec < -10)
            {
                zBas -= 10000;
                stableVec = 0;
            }
            if (zBas > 1000000)
                zBas = 1000000;
            if (zBas < -1000000)
                zBas = -1000000;
            stableCnt++;
            if (stableCnt < 100)
            {
                zSlo = 0;
            }
            if (100 <= stableCnt && stableCnt < 200)
            {
                zSlo += z;
            }
            if (stableCnt == 200)
                zSlo /= 100;
            if (200 < stableCnt)
            {
                stableCnt = 201;
                zSlo = zSlo * 0.98 + z * 0.02;
                z = z * 0.4 + zSlo * 0.6;
            }
            pt++;
            if (pt == 128)
            {
                pt = 0;
								int dif=hForc-targetP[127];
								dif=dif<0?-dif:dif;
								int tf=target*40;
								tf=tf<0?-tf:tf;
								if(dif<tf){
										for (int i = 0; i < 128; i++)
										{
												targetP[i] = hForc + (int)((i + 1.0) * target);
										}
								}else{								
										for (int i = 0; i < 128; i++)
										{
												targetP[i] += (int)((float)(128.0) * target);
										}
								}
            }
            break;
        default:
						z=0;
            break;
        }
        lastError = error;
    }
}

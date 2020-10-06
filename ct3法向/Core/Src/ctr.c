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


//位置一阶误差阈值
extern double sMaxThre;
//位置一阶输出
extern double sMaxOp;
//位置二阶误差阈值
extern double sSndThre;
//位置二阶P
extern double sSndP;
//位置二阶I
extern double sSndI;
//位置三阶I
extern double sTrdI;
//位置三阶D
extern double sTrdD;
//位置三阶I饱和阈值
extern double sTrdDF;
//位置接近误差阈值
extern double sClsThre;
//位置稳定周期数
extern double sStbT;

//启动减速比
extern double SLB;
//速率输出偏移量
extern double dsDif;
//速率输出调整周期
extern double dsChgT;
//速率输出输出调整量
extern double dsChgV;
//速率输出限幅
extern double dsMaxOp;

//力一阶误差阈值
extern double fMaxThre;
//力一阶输出
extern double fMaxOp;
//力二阶误差阈值
extern double fSndThre;
//力二阶P
extern double fSndP;
//力二阶I
extern double fSndI;
//力三阶I
extern double fTrdI;
//力三阶D
extern double fTrdD;
//力置三阶I饱和阈值
extern double fTrdDF;
//力接近误差阈值
extern double fClsThre;
//力变速积分I
extern double fSpcI;
//力去纹波阈值
extern double fReMThre;
//力去纹波P
extern double fReMP;
//力稳定周期数
extern double fStbT;

//力速输出偏移量
extern double dfDif;
//力速输出调整周期
extern double dfChgT;
//力速输出输出调整量
extern double dfChgV;
//力速输出限幅
extern double dfMaxOp;
//力速变速积分周期
extern double dfSpcT;
//力速变速积分比
extern double dfSpcI;
//力速变速积分权重
extern double dfSpcM;
//力速差归零阈值倍数
extern double dfDifZThre;

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
            targetP[0] = hPosi;
            for (int i = 1; i < 128; i++)
            {

                targetP[i] = targetP[i - 1] + (int)(target / SLB);
            }
            lpCnt = 1;
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
            z = 0;
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
            zTrn = (error - lastError) * sTrdD;
            if (zInt > sTrdDF)
                zInt = sTrdDF;
            if (zInt < -sTrdDF)
                zInt = -sTrdDF;
            z = zInt + zTrn;
            if (error < sClsThre && error > -sClsThre)
            {
                //stage 1 wait stablize
                if (stableCnt < sStbT)
                {
                    stableCnt++;
                }
                //stage 2 collect z
                else if (stableCnt < sStbT * 2)
                {
                    zBas += z;
                    stableCnt++;
                    stableVec = 0;
                }
                //stage 3 check z
                else if (stableCnt < sStbT * 3)
                {
                    z = zBas / (sStbT);
                    z = z + error *sTrdI;
                    stableVec += error;
                    stableCnt++;
                }
                //stage 4 stablized
                else
                {
                    if (stableCnt < sStbT * 4 + 100)
                        stableCnt++;
                    if (stableVec > 0)
                    {
                        if (error > 0)
                        {
                            zInt += error * sTrdI;
                            zTrn = (error - lastError) * sTrdD;
                            if (zInt > sTrdDF)
                                zInt = sTrdDF;
                            if (zInt < -sTrdDF)
                                zInt = -sTrdDF;
                            z = zInt + zTrn;
                        }
                        else
                        {
                            z = zBas / (sStbT);
                            z = z + error / 2;
                        }
                    }
                    else
                    {
                        if (error < 0)
                        {
                            zInt += error * sTrdI;
                            zTrn = (error - lastError) * sTrdD;
                            if (zInt > sTrdDF)
                                zInt = sTrdDF;
                            if (zInt < -sTrdDF)
                                zInt = -sTrdDF;
                            z = zInt + zTrn;
                        }
                        else
                        {
                            z = zBas / (sStbT);
                            z = z + error / 2;
                        }
                    }
                }
            }
            else
            {
                if (stableCnt > sStbT * 4)
                {
                    zBas = 0;
                    stableCnt = 0;
                }
            }
            break;
        case 1: //ds
            if (hPosi > targetP[pt])
            {
                z = dsDif + zBas;
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
                z = -dsDif + zBas;
                stableVec--;
            }
            if (stableVec > dsChgT)
            {
                zBas += dsChgV;
                stableVec = 0;
            }
            if (stableVec < -dsChgT)
            {
                zBas -= dsChgV;
                stableVec = 0;
            }
            if (zBas > dsMaxOp)
                zBas = dsMaxOp;
            if (zBas < -dsMaxOp)
                zBas = -dsMaxOp;
            pt++;
            if (pt == 128)
            {
                pt = 0;
                int dif = hPosi - targetP[127];
                dif = dif < 0 ? -dif : dif;
                int tf = target * 10;
                tf = tf < 0 ? -tf : tf;
                lpCnt++;
                if (lpCnt > SLB)
                    lpCnt = SLB;
                if (dif < tf)
                {
                    for (int i = 0; i < 128; i++)
                    {
                        targetP[i] = hPosi + (int)((i + 1.0) * target * (lpCnt / SLB));
                    }
                }
                else
                {
                    for (int i = 0; i < 128; i++)
                    {
                        targetP[i] = targetP[127] + (int)((float)(i + 1) * target * (lpCnt / SLB));
                    }
                }
            }
            break;
        case 2: //f
            if (error > fMaxThre)
            {
                z = fMaxOp;
                break;
            }
            if (error < -fMaxThre)
            {
                z = -fMaxOp;
                break;
            }
            if (error > fSndThre || error < -fSndThre)
            {
                z = fSndP * error;
                zInt = z * fSndI;
                zBas = 0;
                stableCnt = 0;
                break;
            }

            zInt += fTrdI * error;
            zTrn = (error - lastError)*fTrdD;
            if (zInt > fTrdDF)
                zInt = fTrdDF;
            if (zInt < -fTrdDF)
                zInt = -fTrdDF;
            z = zInt + zTrn;
            if (error < fClsThre && error > -fClsThre)
            {
                //stage 1 wait stablize
                if (stableCnt < fStbT)
                {
                    stableCnt++;
                    //stage 2 collect z
                }
                else if (stableCnt < fStbT*2)
                {
                    zBas += z;
                    stableCnt++;
                    stableVec = 0;
                    //stage 3 check z
                }
                else if (stableCnt < fStbT*3)
                {
                    z = zBas / (fStbT);
                    z = z + fTrdI * error;
                    stableVec += error;
                    stableCnt++;
                    zSlo = 0;
                    //stage 4 stablized
                }
                else
                {
                    if (stableCnt < fStbT*4+100)
                        stableCnt++;
                    zSlo += error *fSpcI;
                    if (stableVec > 0)
                    {
                        if (error > fReMThre)
                        {
                            zInt += fTrdI * error;
                            zTrn = (error - lastError)*fTrdD;
                            if (zInt > fTrdDF)
                                zInt = fTrdDF;
                            if (zInt < -fTrdDF)
                                zInt = -fTrdDF;
                            z = zInt + zTrn + zSlo;
                        }
                        else
                        {
                            z = zBas / (fStbT);
                            z = z + fReMP * error + zSlo;
                        }
                    }
                    else
                    {
                        if (error < -fReMThre)
                        {
                            zInt += fTrdI * error;
                            zTrn = (error - lastError)*fTrdD;
                            if (zInt > fTrdDF)
                                zInt = fTrdDF;
                            if (zInt < -fTrdDF)
                                zInt = -fTrdDF;
                            z = zInt + zTrn + zSlo;
                        }
                        else
                        {
                            z = zBas / (fStbT);
                            z = z + fReMP * error + zSlo;
                        }
                    }
                }
            }
            else
            {
                if (stableCnt > fStbT*4)
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
                    z = dfDif + zBas;
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
                    z = -dfDif + zBas;
                else
                    z = 0;
                stableVec--;
            }
            if (stableVec > dfChgT)
            {
                zBas += dfChgV;
                stableVec = 0;
            }
            if (stableVec < -dfChgT)
            {
                zBas -= dfChgV;
                stableVec = 0;
            }
            if (zBas > dfMaxOp)
                zBas = dfMaxOp;
            if (zBas < -dfMaxOp)
                zBas = -dfMaxOp;
            stableCnt++;
            if (stableCnt < dfSpcT)
            {
                zSlo = 0;
            }
            if (dfSpcT <= stableCnt+0.1 && stableCnt < dfSpcT*2-0.1)
            {
                zSlo += z;
            }
            if (dfSpcT*2-0.1<stableCnt && stableCnt < dfSpcT*2+0.1)
                zSlo /= dfSpcT;
            if (dfSpcT*2 < stableCnt+0.1)
            {
                stableCnt = dfSpcT*2+1;
                zSlo = zSlo * (1-dfSpcI) + z * dfSpcI;
                z = z * (1-dfSpcM) + zSlo * dfSpcM;
            }
            pt++;
            if (pt == 128)
            {
                pt = 0;
                int dif = hForc - targetP[127];
                dif = dif < 0 ? -dif : dif;
                int tf = target * dfDifZThre;
                tf = tf < 0 ? -tf : tf;
                if (dif < tf)
                {
                    for (int i = 0; i < 128; i++)
                    {
                        targetP[i] = hForc + (int)((i + 1.0) * target);
                    }
                }
                else
                {
                    for (int i = 0; i < 128; i++)
                    {
                        targetP[i] += (int)((float)(128.0) * target);
                    }
                }
            }
            break;
        default:
            z = 0;
            break;
        }
        lastError = error;
    }
}

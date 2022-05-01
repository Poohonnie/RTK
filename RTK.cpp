#include "RTKLib.h"
#include "SPP.h"
#include "RTK.h"
#include <cmath>

void RTK::SelectRefSat()
{
    // 选取参考星实际上只对流动站观测值进行选取
    memset(&ddObs.selected, 0, sizeof(bool) * 4);// 标记本历元参考星未选取成功
    
    // 检查上一历元参考星是否满足条件
    for (int k = 0; k < 2; ++k)
    {
        GNSS sys = (GNSS) k;
        int prn = ddObs.refSatPrn[k];  // 上一历元参考星prn号
        if (prn < 1)  // 说明上一历元没有参考星? 可能说明是第一个历元
            continue;
        
        for (int i = 0; i < sdObs.sdNum; ++i)
        {
            if (sdObs.satSd[i].prn != prn || sdObs.satSd[i].sys != sys || !sdObs.satSd[i].valid)
                // 说明这颗卫星不是上一历元的参考星 或 观测值不可用
                continue;
            // 将上一历元参考星继承至这个历元
            int rIndex = spp[0].epkPos.FindSatPosIndex(prn, sys);  // 卫星定位结果, 求单差的时候已经确认一定有结果
            if (sdObs.satSd[i].valid && spp[0].epkPos.satPos[rIndex].eleAngle > 40 * constant::D2R)
            {
                // 高度角大于40°则继承
                ddObs.refSatIndex[k] = i;
                ddObs.refSatPrn[k] = prn;
                ddObs.selected[k] = true;
            }
        }
    }
    
    for (int k = 0; k < 2; ++k)
    {
        
        if (ddObs.refSatPrn[k] == sdObs.satSd[ddObs.refSatIndex[k]].prn &&
            sdObs.satSd[ddObs.refSatIndex[k]].sys == (GNSS) k)
            // 参考星已选取好, 即继承自上个历元
            continue;
        
        double maxEleAngle = 40 * constant::D2R;  // 当前最大的高度角, 初始为40°
        for (int i = 0; i < sdObs.sdNum; ++i)
        {
            // 对单差观测值sdObs进行遍历
            // epkObs, epkPos, satSd, gfmw存储的卫星顺序和数目全部相同
            // 计算单差的时候已经确认过卫星位置计算完毕
            
            int prn = sdObs.satSd[i].prn;
            GNSS sys = sdObs.satSd[i].sys;
            if (sys != (GNSS) k)
                // 当前单差观测值卫星系统和要选参考星的卫星系统不符
                continue;
            int rIndex = spp[0].epkPos.FindSatPosIndex(prn, sys);  // 流动站epkPos索引下标
            int bIndex = spp[1].epkPos.FindSatPosIndex(prn, sys);  // 基准站epkPos索引下标
            if (!sdObs.satSd[i].valid || !spp[0].epkPos.satPos[rIndex].valid || !spp[1].epkPos.satPos[bIndex].valid)
                // 当前卫星单差观测值 以及 流动站和基准站卫星定位结果 其中有不可用数据时
                continue;
            if (spp[0].epkPos.satPos[rIndex].eleAngle > maxEleAngle)
            {
                // 该卫星高度角目前最大
                // 记录最大高度角以及存储参考星索引号, 如果后面发现有更大的则再进行更换
                // 因为这里已经有单差了, 默认流动站和基站都有观测值, 所以只要看流动站观测值结果
                maxEleAngle = spp[0].epkPos.satPos[rIndex].eleAngle;
                ddObs.refSatIndex[(int) sys] = i;
                ddObs.refSatPrn[(int) sys] = prn;
                ddObs.selected[k] = true;
            }
        }
    }
}

void DDObs::GetDDObs(const SDObs &sdObs)
{
    // 清空上一历元双差观测值, 保留参考星选取信息
    memset(&this->ddPrn, 0, MAXCHANNELNUM * sizeof(int));
    memset(&this->ddSys, 0, MAXCHANNELNUM * sizeof(GNSS));
    memset(&this->ddNum, 0, sizeof(int));
    memset(&this->sysNum, 0, 4 * sizeof(int));
    memset(&this->dd, 0, MAXCHANNELNUM * 4 * sizeof(double));
    memset(&this->fixedAmb, 0, MAXCHANNELNUM * 2 * sizeof(double));
    
    
    for (int i = 0; i < sdObs.sdNum; ++i)
    {
        // 对单差观测值进行循环
        GNSS sys = sdObs.satSd[i].sys;
        if (!selected[(int) sys])
            // 说明该系统没有参考星
            continue;
        int prn = sdObs.satSd[i].prn;
        int ref = refSatIndex[(int) sys];  // 该卫星系统参考星下标
        if (i == ref || !sdObs.satSd[i].valid)
            // 说明当前卫星为参考星 或 单差观测值不可用
            continue;
        // 当前卫星不为参考星, 则与参考星观测值作双差
        // 不刻意将GPS和BDS双差分开, 混合存储
        // 注意是当前卫星 - 参考星
        dd[ddNum][0] = sdObs.satSd[i].psrSD[0] - sdObs.satSd[ref].psrSD[0];
        dd[ddNum][1] = sdObs.satSd[i].psrSD[1] - sdObs.satSd[ref].psrSD[1];
        dd[ddNum][2] = sdObs.satSd[i].cpSD[0] - sdObs.satSd[ref].cpSD[0];
        dd[ddNum][3] = sdObs.satSd[i].cpSD[1] - sdObs.satSd[ref].cpSD[1];
        
        // 记录卫星系统及卫星号, 方便后面读取spp中的卫星位置
        ddPrn[ddNum] = prn;
        ddSys[ddNum] = sys;
        
        ++ddNum;  // 可用双差观测值数目+1
        ++sysNum[(int) sys];  // 不同卫星系统双差观测值数目+1
    }
}

int RTK::CalFixedSolution(RAWDATA &roverRaw, RAWDATA &baseRaw, EPKGFMW &rEpkGfmw, EPKGFMW &bEpkGfmw, CONFIG &config)
{
    // 初始化, 清空上一历元结果
    memset(&pos, 0, sizeof(XYZ));
    memset(&resAmb, 0, sizeof(double) * 2);
    valid = false;
    sol = 0;
    ratio = 0.0;
    
    t = roverRaw.epkObs.t;  // 确定观测时间
    int iter_fix{};  // 固定解迭代次数
    
    // 设置流动站和基准站位置初值
    // 注意spp[0]为rover流动站; spp[1]为base基准站
    int spp0 = spp[0].StdPntPos(roverRaw, rEpkGfmw, config);
    int spp1 = spp[1].StdPntPos(baseRaw, bEpkGfmw, config);
    if (!spp0)  // 单点解
    {
        valid = true;
        sol = 0;
    }
    pos = spp[0].sttnXyz;  // 浮点解结果初始化
    XYZ basePos = {-2267804.5263, 5009342.3723, 3220991.8632}/*spp[1].sttnXyz*/;  // 基站坐标
    
    if (spp0 == -114514 || spp1 == -114514)
    {
        // 流动站或基站其中某一站SPP解算失败
        printf("%4d %10.3f SPP failed!\n", t.week, t.secOfWeek);
        return -114514;
    }
    
    // 获取单差观测值
    sdObs.GetSDObs(roverRaw.epkObs, baseRaw.epkObs, spp[0], spp[1]);
    // 单差观测值周跳探测
    detectSlip.DetectCycleSlip(sdObs);
    
    do
    {
        sol = 0;  // 解类型初始化, 目前只有单点解
        // 以流动站rover为准选取参考星
        SelectRefSat();
        // 获取双差观测值
        ddObs.GetDDObs(sdObs);  // 没有参考星的系统不进行双差计算
        
        if (!ddObs.selected[0] && !ddObs.selected[1])
        {
            // GPS和BDS均没有参考星
            printf("%4d %10.3f Fail to select reference satellite!\n", t.week, t.secOfWeek);
            return -114514;
        }
        if (ddObs.sysNum[0] + ddObs.sysNum[1] < 5)
        {
            // 双差观测值数少于5就不进行计算了
            printf("%4d %10.3f Have no enough ddObs. GPS: %2d, BDS: %2d.\n",
                   t.week, t.secOfWeek, ddObs.sysNum[0], ddObs.sysNum[1]);
            return -114514;
        }
        
        // 计算基准站到所有卫星的几何距离
        double dBaseSats[MAXCHANNELNUM][4]{};
        for (int i = 0; i < ddObs.ddNum; ++i)
        {
            // 在spp[1].epkPos.satPos找出参与双差计算的卫星, 计算到基站的几何距离
            int bIndex = spp[1].epkPos.FindSatPosIndex(ddObs.ddPrn[i], ddObs.ddSys[i]);
            // 计算双差的时候已经确认过了在spp中存在该卫星, 所以不用考虑未找到的情况
            SatPos satPos = spp[1].epkPos.satPos[bIndex];  // 方便书写
            // 计算基站到卫星的距离
            dBaseSats[i][0] = basePos.x - satPos.satXyz.x;
            dBaseSats[i][1] = basePos.y - satPos.satXyz.y;
            dBaseSats[i][2] = basePos.z - satPos.satXyz.z;
            dBaseSats[i][3] = sqrt(dBaseSats[i][0] * dBaseSats[i][0]
                                   + dBaseSats[i][1] * dBaseSats[i][1] + dBaseSats[i][2] * dBaseSats[i][2]);
        }
        // 计算基准站到参考星的几何距离
        double dBaseRef[4][4]{};
        for (int k = 0; k < 2; ++k)
        {
            if (!ddObs.selected[k])  // 说明该系统没有参考星
                continue;
            // 找出参考星对应的卫星位置
            int bIndex = spp[1].epkPos.FindSatPosIndex(ddObs.refSatPrn[k], (GNSS) k);
            SatPos satPos = spp[1].epkPos.satPos[bIndex];  // 方便书写
            // 计算基站到参考星的距离
            dBaseRef[k][0] = basePos.x - satPos.satXyz.x;
            dBaseRef[k][1] = basePos.y - satPos.satXyz.y;
            dBaseRef[k][2] = basePos.z - satPos.satXyz.z;
            dBaseRef[k][3] = sqrt(dBaseRef[k][0] * dBaseRef[k][0]
                                  + dBaseRef[k][1] * dBaseRef[k][1] + dBaseRef[k][2] * dBaseRef[k][2]);
        }
        
        // 最小二乘迭代
        // 矩阵维数
        int rowNum = 4 * ddObs.ddNum;  // B矩阵行数(4*n), 即观测方程数
        int colNum = 2 * ddObs.ddNum + 3;  // B矩阵列数(2*n+3)
        
        CMatrix B(rowNum, colNum);  // B矩阵
        CMatrix BTPB_inv(colNum, colNum);  // BTPB的逆
        CMatrix W(rowNum, 1);  // W矩阵
        CMatrix dX(colNum, 1);  // 迭代改正项 3基线向量 + 2n模糊度
        CMatrix dX0(colNum, 1);  // 基线向量
        CMatrix Pxx(rowNum, rowNum);  // 4n*4n 的权阵
        dX0.Write(pos.x - basePos.x, 0, 0);  // 基线向量初始化
        dX0.Write(pos.y - basePos.y, 1, 0);
        dX0.Write(pos.z - basePos.z, 2, 0);
        
        int iter{};  // 迭代次数
        double norm{};  // 基线迭代增量
        do
        {
            // B矩阵和W矩阵置零
            B.SetZero();
            W.SetZero();
            // 计算流动站到所有卫星的几何距离
            double dRovSats[MAXCHANNELNUM][4]{};
            for (int i = 0; i < ddObs.ddNum; ++i)
            {
                // 在spp[0].epkPos.satPos找出参与双差计算的卫星, 计算到基站的几何距离
                int rIndex = spp[0].epkPos.FindSatPosIndex(ddObs.ddPrn[i], ddObs.ddSys[i]);
                // 计算双差的时候已经确认过了在spp中存在该卫星, 所以不用考虑未找到的情况
                SatPos satPos = spp[0].epkPos.satPos[rIndex];  // 方便书写
                // 计算基站到卫星的距离
                dRovSats[i][0] = pos.x - satPos.satXyz.x;
                dRovSats[i][1] = pos.y - satPos.satXyz.y;
                dRovSats[i][2] = pos.z - satPos.satXyz.z;
                dRovSats[i][3] = sqrt(dRovSats[i][0] * dRovSats[i][0]
                                      + dRovSats[i][1] * dRovSats[i][1] + dRovSats[i][2] * dRovSats[i][2]);
            }
            
            // 计算流动站到参考星的几何距离
            double dRovRef[4][4]{};
            for (int k = 0; k < 2; ++k)
            {
                if (!ddObs.selected[k])  // 说明该系统没有参考星
                    continue;
                // 找出参考星对应的卫星位置
                int rIndex = spp[0].epkPos.FindSatPosIndex(ddObs.refSatPrn[k], (GNSS) k);
                SatPos satPos = spp[0].epkPos.satPos[rIndex];  // 方便书写
                // 计算流动站到参考星的距离
                dRovRef[k][0] = pos.x - satPos.satXyz.x;
                dRovRef[k][1] = pos.y - satPos.satXyz.y;
                dRovRef[k][2] = pos.z - satPos.satXyz.z;
                dRovRef[k][3] = sqrt(dRovRef[k][0] * dRovRef[k][0]
                                     + dRovRef[k][1] * dRovRef[k][1] + dRovRef[k][2] * dRovRef[k][2]);
            }
            
            // 线性化双差观测方程得到B矩阵和W矩阵
            for (int i = 0; i < ddObs.ddNum; ++i)
            {
                int sysId1 = (int) ddObs.ddSys[i];  // 卫星系统号 0:GPS 1:BDS
                double lR = dRovSats[i][0] / dRovSats[i][3] - dRovRef[sysId1][0] / dRovRef[sysId1][3];
                double mR = dRovSats[i][1] / dRovSats[i][3] - dRovRef[sysId1][1] / dRovRef[sysId1][3];
                double nR = dRovSats[i][2] / dRovSats[i][3] - dRovRef[sysId1][2] / dRovRef[sysId1][3];
                
                // 一个双差观测值占4行, 所以行数是4*i
                // +j的意思是P1 P2 L1 L2这四个
                for (int j = 0; j < 4; ++j)
                {
                    // 分别写入lR, mR, nR
                    B.Write(lR, 4 * i + j, 0);
                    B.Write(mR, 4 * i + j, 1);
                    B.Write(nR, 4 * i + j, 2);
                }
                // 第2, 3行是相位观测值, 需要加模糊度的系数
                B.Write(sysId1 == 0 ? constant::wl_l1 : constant::wl_b1, 4 * i + 2, 3 + 2 * i + 0);
                B.Write(sysId1 == 0 ? constant::wl_l2 : constant::wl_b3, 4 * i + 3, 3 + 2 * i + 1);
                
                // 残差向量
                double rho = dRovSats[i][3] - dRovRef[sysId1][3] - dBaseSats[i][3] + dBaseRef[sysId1][3];
                double w0 = ddObs.dd[i][0] - rho;
                double w1 = ddObs.dd[i][1] - rho;
                double w2 = ddObs.dd[i][2] - rho;
                double w3 = ddObs.dd[i][3] - rho;
                W.Write(w0, 4 * i + 0, 0);
                W.Write(w1, 4 * i + 1, 0);
                W.Write(w2, 4 * i + 2, 0);
                W.Write(w3, 4 * i + 3, 0);
                
                // 构造权矩阵
                if (iter == 0)
                {
                    // 目前循环处于第4*i+0 ~ 4*i+3行
                    for (int j = 0; j < ddObs.ddNum; ++j)
                    {
                        // 4*i+0~4*i+3行, 4*j+0~4*j+3列
                        // 双差伪距和相位的权矩阵, GPS伪距1m, BDS伪距2m, GPS相位1mm, BDS相位2mm的权重
                        int sysId2 = (int) ddObs.ddSys[j];  // 卫星系统编号
                        
                        if (j == i)
                        {
                            // 行数等于列数, 也就是处在对角线上的4*4分块
                            // 4*4块内对角线元素为n, 非对角线元素都为0, 不用额外写
                            double scale = 1.0 * ddObs.sysNum[sysId2] / (ddObs.sysNum[sysId2] + 1.0);  // 方便书写
                            switch (sysId2)
                            {
                                case 0/*GPS*/:
                                    Pxx.Write(1.0000 * scale, 4 * i + 0, 4 * j + 0);  // P1
                                    Pxx.Write(1.0000 * scale, 4 * i + 1, 4 * j + 1);  // P2
                                    Pxx.Write(1000.0 * scale, 4 * i + 2, 4 * j + 2);  // L1
                                    Pxx.Write(1000.0 * scale, 4 * i + 3, 4 * j + 3);  // L2
                                    break;
                                case 1/*BDS*/:
                                    Pxx.Write(0.500 * scale, 4 * i + 0, 4 * j + 0);  // P1
                                    Pxx.Write(0.500 * scale, 4 * i + 1, 4 * j + 1);  // P2
                                    Pxx.Write(500.0 * scale, 4 * i + 2, 4 * j + 2);  // L1
                                    Pxx.Write(500.0 * scale, 4 * i + 3, 4 * j + 3);  // L2
                                    break;
                                default:
                                    break;
                            }
                        } else
                        {
                            // 不在主对角线上的4*4分块
                            // 4*4块内非对角线元素都为0
                            if (sysId1 != sysId2)
                                // 卫星系统不同, 分块对角线元素是0
                                continue;
                            // 所在的行列卫星系统都相同, 对角线元素为-1
                            switch (sysId2)
                            {
                                case 0/*GPS*/:
                                    Pxx.Write(-1.0000 / (ddObs.sysNum[sysId2] + 1.0), 4 * i + 0, 4 * j + 0);  // P1
                                    Pxx.Write(-1.0000 / (ddObs.sysNum[sysId2] + 1.0), 4 * i + 1, 4 * j + 1);  // P2
                                    Pxx.Write(-1000.0 / (ddObs.sysNum[sysId2] + 1.0), 4 * i + 2, 4 * j + 2);  // L1
                                    Pxx.Write(-1000.0 / (ddObs.sysNum[sysId2] + 1.0), 4 * i + 3, 4 * j + 3);  // L2
                                    break;
                                case 1/*BDS*/:
                                    Pxx.Write(-0.500 / (ddObs.sysNum[sysId2] + 1.0), 4 * i + 0, 4 * j + 0);  // P1
                                    Pxx.Write(-0.500 / (ddObs.sysNum[sysId2] + 1.0), 4 * i + 1, 4 * j + 1);  // P2
                                    Pxx.Write(-500.0 / (ddObs.sysNum[sysId2] + 1.0), 4 * i + 2, 4 * j + 2);  // L1
                                    Pxx.Write(-500.0 / (ddObs.sysNum[sysId2] + 1.0), 4 * i + 3, 4 * j + 3);  // L2
                                    break;
                                default:
                                    break;
                            }
                        }
                    }
                }
            }  // 线性化结束
            // 最小二乘解算
            CMatrix BT = B.Trans();  // B转置
            CMatrix BTP = BT * Pxx;
            CMatrix BTPB = BTP * B;
            BTPB_inv = BTPB.Inv();  // BTPB的逆  不可逆的话返回的是单位阵
            CMatrix BTPl = BTP * W;
            dX = BTPB_inv * BTPl;  // 改正项
            dX0 += dX;  // 基线向量更新
            // 更新测站坐标
            pos.x = basePos.x + dX0.Read(0, 0);
            pos.y = basePos.y + dX0.Read(1, 0);
            pos.z = basePos.z + dX0.Read(2, 0);
            
            norm = sqrt(dX.Read(0, 0) * dX.Read(0, 0)
                        + dX.Read(1, 0) * dX.Read(1, 0)
                        + dX.Read(2, 0) * dX.Read(2, 0));  // 变化量
            ++iter;
        } while (norm > 1e-5 && iter < 5);
        if (iter > 4)  // 说明没有正常收敛
            return -114514;
        sol = 1;  // 浮点解可用
        
        // LAMBDA模糊度固定
        CMatrix QNN = BTPB_inv;
        // QXX阵去掉三行三列, 即去掉跟坐标相关的量, 只留下跟模糊度相关的量
        for (int i = 0; i < 3; ++i)
        {
            QNN.SubRow(0);
            QNN.SubCol(0);
        }
        // 模糊度固定
        if (lambda(colNum - 3, 2, dX.mat + 3, QNN.mat, ddObs.fixedAmb, resAmb) != 0)
        {
            printf("%4d %10.3f Lambda failed. Sats: %d.\n", t.week, t.secOfWeek, ddObs.ddNum);
            return -114514;  // != 0就是固定失败
        }
        
        ratio = resAmb[0] > 0.0 ? resAmb[1] / resAmb[0] : 1.0;  // ratio 值
        ratio = ratio > 999.99 ? 999.99 : ratio;  // 大于999.0设为999.0
        if (ratio < 2.0)
        {
            // 考虑是否有半周问题
            for (int i = 0; i < ddObs.ddNum; ++i)
            {
                if (fabs(fabs(ddObs.fixedAmb[2 * i] - ddObs.fixedAmb[2 * i + colNum - 3]) - 1.0) < 0.5 ||
                    fabs(fabs(ddObs.fixedAmb[2 * i + 1] - ddObs.fixedAmb[2 * i + 1 + colNum - 3]) - 1.0) < 0.5)
                    sdObs.satSd[i].valid = false;
            }
        }
        for (int i = 0; i < ddObs.ddNum; ++i)
        {
            double deltaw1 = B.Read(4 * i + 2, 3 + 2 * i + 0)
                             * (ddObs.fixedAmb[2 * i + 0]/*整数模糊度*/ - dX0.Read(3 + 2 * i + 0, 0/*浮点模糊度*/));
            double deltaw2 = B.Read(4 * i + 3, 3 + 2 * i + 1)
                             * (ddObs.fixedAmb[2 * i + 1]/*整数模糊度*/ - dX0.Read(3 + 2 * i + 1, 0/*浮点模糊度*/));
            
            W.Write(W.Read(4 * i + 2, 0) + deltaw1, 4 * i + 2, 0);
            W.Write(W.Read(4 * i + 3, 0) + deltaw2, 4 * i + 3, 0);
        }
        // 求解固定解
        while (B.cols > 3)  // 待求量只有基线向量 3列
            B.SubCol(B.cols - 1);
        while (dX0.rows > 3)  // 基线向量只含坐标变量 也就是3行
            dX0.SubRow(dX0.rows - 1);
        
        CMatrix BT = B.Trans();  // B转置
        CMatrix BTPB = BT * Pxx * B;
        BTPB_inv = BTPB.Inv();  // BTPB的逆  不可逆的话返回的是单位阵
        CMatrix BTPl = BT * Pxx * W;
        dX = BTPB_inv * BTPl;  // 改正项
        dX0 -= dX;  // 基线向量更新
        // 更新测站坐标
        pos.x = basePos.x + dX0.Read(0, 0);
        pos.y = basePos.y + dX0.Read(1, 0);
        pos.z = basePos.z + dX0.Read(2, 0);
        ++iter_fix;
    } while (iter_fix < 2 && ratio < config.ratioThres);
    if (ratio >= config.ratioThres)
    {
        sol = 2;  // 固定解可用
        return 0;  // 正常返回 浮点解结算成功
    } else
        return -114514;  // ratio值过小, 固定失败
}

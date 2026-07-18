#include "LidarIris.h"
#include "fftm/fftm.hpp"

cv::Mat1b LidarIris::GetIris(const pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    // 80 径向 360 角度
    cv::Mat1b IrisMap = cv::Mat1b::zeros(80, 360);

    for (pcl::PointXYZ p : cloud.points)
    {
        // dis not include z
        float dis = sqrt(p.data[0] * p.data[0] + p.data[1] * p.data[1]);
        float yaw = (atan2(p.data[1], p.data[0]) * 180.0f / M_PI) + 180;
        int Q_dis = std::min(std::max((int)floor(dis), 0), 79);
        int Q_arc = std::min(std::max((int)ceil(p.z + 5), 0), 7);
        int Q_yaw = std::min(std::max((int)floor(yaw + 0.5), 0), 359);
        IrisMap.at<uint8_t>(Q_dis, Q_yaw) |= (1 << Q_arc);
    }

    return IrisMap;
}


float LidarIris::Compare(const LidarIris::FeatureDesc &img1, const LidarIris::FeatureDesc &img2, int *bias)
{
    // FFTMatch 可以匹配任意角度，但是对于超过 180° 的翻转，最好提前翻转一次，否则误差很大
    if(_matchNum==2) // 正向反向都有
    {
        // 正向匹配
        auto firstRect = FFTMatch(img2.img, img1.img);
        int firstShift = firstRect.center.x - img1.img.cols / 2;
        float dis1;
        int bias1;
        GetHammingDistance(img1.T, img1.M, img2.T, img2.M, firstShift, dis1, bias1);

        // 反向匹配
        auto T2x = circShift(img2.T, 0, 180);
        auto M2x = circShift(img2.M, 0, 180);
        auto img2x = circShift(img2.img, 0, 180);

        auto secondRect = FFTMatch(img2x, img1.img);
        int secondShift = secondRect.center.x - img1.img.cols / 2;
        float dis2 = 0;
        int bias2 = 0;
        GetHammingDistance(img1.T, img1.M, T2x, M2x, secondShift, dis2, bias2);

        if (dis1 < dis2)
        {
            if (bias)
                *bias = bias1;
            return dis1;
        }
        else
        {
            if (bias)
                *bias = (bias2 + 180) % 360;
            return dis2;
        }
    }
    if(_matchNum==1) // 只有反向
    {
        auto T2x = circShift(img2.T, 0, 180);
        auto M2x = circShift(img2.M, 0, 180);
        auto img2x = circShift(img2.img, 0, 180);

        auto secondRect = FFTMatch(img2x, img1.img);
        int secondShift = secondRect.center.x - img1.img.cols / 2;
        float dis2 = 0;
        int bias2 = 0;
        GetHammingDistance(img1.T, img1.M, T2x, M2x, secondShift, dis2, bias2);
        if (bias)
            *bias = (bias2 + 180) % 360;
        return dis2;
    }
    if(_matchNum==0) // 只有正向
    {
        auto firstRect = FFTMatch(img2.img, img1.img);
        int firstShift = firstRect.center.x - img1.img.cols / 2;
        float dis1;
        int bias1;
        GetHammingDistance(img1.T, img1.M, img2.T, img2.M, firstShift, dis1, bias1);
        if (bias)
            *bias = bias1;
        return dis1;

    }
    return NAN;
}

/**
 * @brief 对 LiDAR-Iris 图像的每一行做 LoG-Gabor 滤波，提取不同频率的纹理特征
 * @param src 输入 80 * 360 的LiDAR-Iris 原始图像
 * @param nscale 滤波器尺度数（e.p. 4）
 * @param minWaveLength 最小波长（e.p. 3）
 * @param mult 尺度间波长倍数
 * @param sigmaOnf 高斯函数带宽
 * @return std::vector<cv::Mat2f> 返回 nscale 个复数矩阵，每个是滤波响应
 */
std::vector<cv::Mat2f> LidarIris::LogGaborFilter(const cv::Mat1f &src, unsigned int nscale, int minWaveLength, double mult, double sigmaOnf)
{
    int rows = src.rows; // 80
    int cols = src.cols; // 360
    cv::Mat2f filtersum = cv::Mat2f::zeros(1, cols);
    std::vector<cv::Mat2f> EO(nscale);
    int ndata = cols;
    if (ndata % 2 == 1)
        ndata--;
    cv::Mat1f logGabor = cv::Mat1f::zeros(1, ndata);
    cv::Mat2f result = cv::Mat2f::zeros(rows, ndata);
    cv::Mat1f radius = cv::Mat1f::zeros(1, ndata / 2 + 1);
    radius.at<float>(0, 0) = 1;
    for (int i = 1; i < ndata / 2 + 1; i++)
    {
        radius.at<float>(0, i) = i / (float)ndata;
    }
    double wavelength = minWaveLength;
    for (int s = 0; s < nscale; s++)
    {
        double fo = 1.0 / wavelength;
        cv::Mat1f temp; //(radius.size());
        cv::log(radius / fo, temp);
        cv::pow(temp, 2, temp);
        cv::exp((-temp) / (2 * log(sigmaOnf) * log(sigmaOnf)), temp);
        temp.copyTo(logGabor.colRange(0, ndata / 2 + 1));
        //
        logGabor.at<float>(0, 0) = 0;
        cv::Mat2f filter;
        cv::Mat1f filterArr[2] = {logGabor, cv::Mat1f::zeros(logGabor.size())};
        cv::merge(filterArr, 2, filter);
        filtersum = filtersum + filter;
        for (int r = 0; r < rows; r++)
        {
            cv::Mat2f src2f;
            cv::Mat1f srcArr[2] = {src.row(r).clone(), cv::Mat1f::zeros(1, src.cols)};
            cv::merge(srcArr, 2, src2f);
            cv::dft(src2f, src2f);
            cv::mulSpectrums(src2f, filter, src2f, 0);
            cv::idft(src2f, src2f);
            src2f.copyTo(result.row(r));
        }
        EO[s] = result.clone();
        wavelength *= mult;
    }
    filtersum = circShift(filtersum, 0, cols / 2);
    return EO;
}

void LidarIris::LoGFeatureEncode(const cv::Mat1b &src, unsigned int nscale, int minWaveLength, double mult, double sigmaOnf, cv::Mat1b &T, cv::Mat1b &M)
{
    cv::Mat1f srcFloat;
    src.convertTo(srcFloat, CV_32FC1);
    auto list = LogGaborFilter(srcFloat, nscale, minWaveLength, mult, sigmaOnf);
    std::vector<cv::Mat1b> Tlist(nscale * 2), Mlist(nscale * 2);
    for (int i = 0; i < list.size(); i++)
    {
        cv::Mat1f arr[2];
        cv::split(list[i], arr);
        // 实部 - 正相(cosine > 0) 纹理为true -> 255
        Tlist[i] = arr[0] > 0;
        // 虚部 - 正交(sine > 0) 纹理为true -> 255
        Tlist[i + nscale] = arr[1] > 0;
        cv::Mat1f m; // 掩码 - 无效纹理直接调过计算
        cv::magnitude(arr[0], arr[1], m);
        Mlist[i] = m < 0.0001;
        Mlist[i + nscale] = m < 0.0001;
    }
    // vconcat 用于拼接矩阵
    cv::vconcat(Tlist, T);
    cv::vconcat(Mlist, M);
}

LidarIris::FeatureDesc LidarIris::GetFeature(const cv::Mat1b &src)
{
    FeatureDesc desc;
    desc.img = src.clone();
    LoGFeatureEncode(src, _nscale, _minWaveLength, _mult, _sigmaOnf, desc.T, desc.M);
    return desc;
}

/**
 * @brief 计算两帧汉明码
 * @param T1 帧1描述子
 * @param M1 帧1掩码
 * @param T2 帧2描述子
 * @param M2 帧2掩码
 * @param scale FFT 估计的偏移量，作为搜索起点
 * @param dis 计算的汉明距离
 * @param bias 实际最优偏移量，实际搜索会在 scale 附近进行一次轮询偏移量搜索（因为scale可能不是最优）
 * @return void
 */
void LidarIris::GetHammingDistance(const cv::Mat1b &T1, const cv::Mat1b &M1, const cv::Mat1b &T2, const cv::Mat1b &M2, int scale, float &dis, int &bias)
{
    dis = NAN;
    bias = -1;
    for (int shift = scale - 2; shift <= scale + 2; shift++)
    {
        cv::Mat1b T1s = circShift(T1, 0, shift);
        cv::Mat1b M1s = circShift(M1, 0, shift);
        cv::Mat1b mask = M1s | M2;
        int MaskBitsNum = cv::sum(mask / 255)[0];
        int totalBits = T1s.rows * T1s.cols - MaskBitsNum;
        cv::Mat1b C = T1s ^ T2;
        C = C & ~mask;
        int bitsDiff = cv::sum(C / 255)[0];
        if (totalBits == 0)
        {
            dis = NAN;
        }
        else
        {
            float currentDis = bitsDiff / (float)totalBits;
            if (currentDis < dis || isnan(dis))
            {
                dis = currentDis;
                bias = shift;
            }
        }
    }
    return;
}

inline cv::Mat LidarIris::circRowShift(const cv::Mat &src, int shift_m_rows)
{
    if (shift_m_rows == 0)
        return src.clone();
    shift_m_rows %= src.rows;
    int m = shift_m_rows > 0 ? shift_m_rows : src.rows + shift_m_rows;
    cv::Mat dst(src.size(), src.type());
    src(cv::Range(src.rows - m, src.rows), cv::Range::all()).copyTo(dst(cv::Range(0, m), cv::Range::all()));
    src(cv::Range(0, src.rows - m), cv::Range::all()).copyTo(dst(cv::Range(m, src.rows), cv::Range::all()));
    return dst;
}

inline cv::Mat LidarIris::circColShift(const cv::Mat &src, int shift_n_cols)
{
    if (shift_n_cols == 0)
        return src.clone();
    shift_n_cols %= src.cols;
    int n = shift_n_cols > 0 ? shift_n_cols : src.cols + shift_n_cols;
    cv::Mat dst(src.size(), src.type());
    src(cv::Range::all(), cv::Range(src.cols - n, src.cols)).copyTo(dst(cv::Range::all(), cv::Range(0, n)));
    src(cv::Range::all(), cv::Range(0, src.cols - n)).copyTo(dst(cv::Range::all(), cv::Range(n, src.cols)));
    return dst;
}

cv::Mat LidarIris::circShift(const cv::Mat &src, int shift_m_rows, int shift_n_cols)
{
    return circColShift(circRowShift(src, shift_m_rows), shift_n_cols);
}

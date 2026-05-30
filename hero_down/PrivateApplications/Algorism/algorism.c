/** Include Headder Files **/
#include "algorism.h"
#include "math.h"

/**
 * \brief 平滑滤波器初始化
 * \param[in] alpha 滤波系数
 */
void SmoothFilterInitialize(SmoothFilter* filter, float alpha)
{
    filter->alpha = alpha;
		filter->current = filter->last = 0;
}

/**
 * \brief 平滑滤波器更新 
 */
float SmoothFilterUpdate(SmoothFilter* filter, float input)
{
    filter->last = filter->current;
    return (filter->current = filter->alpha * filter->last + \
                              (1 - filter->alpha) * input);
}

void AverageFilterInitialize(AverageFilter*filter)
{
	filter->last[0]=0;
	filter->last[1]=0;
	filter->last[2]=0;
}

float AverageFilterUpdate(AverageFilter*filter, float input)
{
	input*=0.1;
	input+=0.4*filter->last[2];
	input+=0.3*filter->last[1];
	input+=0.2*filter->last[0];
	for(int i=2;i>0;i--)
		filter->last[i]=filter->last[i-1];
	filter->last[0]=input;
	return filter->last[0];
}

/**
 * \brief 角度限制
 * \param[in] angle 输入角度
 * \param[in] limitMIN 角度上限
 * \param[in] limitMAX 角度下限
 * \return 限幅后的角度值 
 */
float AngleLimit(float angle, float limit_min, float limit_max)
{
	if(limit_max == limit_min) 
		return angle;
	float stride = limit_max - limit_min;
	while  (angle < limit_min)
		angle += stride;
	while (angle > limit_max)
		angle -= stride;
	return angle;
}

/**
 * @brief 正负最大值限制
 */
float AbsLimiter(float val, float max)
{
	if(val > max) 
	{
		val = max;
		return val;
	}
	if(val < -max)
		val = -max;
	
	return val;
}

 
/**
 * @brief 双边限幅
 */float DoubleEdgeLimiter(float val, float min, float max)
{
	return (val > max ? max : (val < min ? min : val));
}

/**
  * @brief  fast inverse square-root, to calculate 1/Sqrt(x)
  * @param  x: the number need to be calculated
  * @retval 1/Sqrt(x)
  * @usage  call in OnboardIMUahrsUpdate() function
  */
float InvSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long *)&y;

	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	y = y * (1.5f - (halfx * y * y));

	return y;
}

/**
 * @brief 符号函数
 */
int Sign(float x){
	if (x>1e-9)
		return 1;
	else if(x<-1e-9)
		return -1;
	else
		return 0;
}

/**
 * @brief 最小二乘法
 */
void leastSquareLinearFit(leastSquareLinear* data)
{
	
	
    float sum_x2 = 0.0;
    float sum_y  = 0.0;
    float sum_x  = 0.0;
    float sum_xy = 0.0;



	for (int i = 0; i < data->valid_num; ++i) {
		sum_x2 += data->x[i]*data->x[i];
		sum_y  += data->y[i];
		sum_x  += data->x[i];
		sum_xy += data->x[i]*data->y[i];
	}


    data->a = (data->valid_num*sum_xy - sum_x*sum_y)/(data->valid_num*sum_x2 - sum_x*sum_x);
    data->b = (sum_x2*sum_y - sum_x*sum_xy)/(data->valid_num*sum_x2-sum_x*sum_x);

}

void improvedleastSquareLinearFit(leastSquareLinear* data, float lambda) {
    // 检查输入有效性
    if (data->num <= 0) return;
    if (lambda <= 0.0f || lambda > 1.0f) {
        // 处理无效的lambda值，可以选择返回错误或使用默认值
        lambda = 1.0f; // 默认使用普通最小二乘法
    }

    float sum_w = 0.0f;
    float sum_wx = 0.0f;
    float sum_wy = 0.0f;
    float sum_wx2 = 0.0f;
    float sum_wxy = 0.0f;
    
    // 修正权重计算顺序：最新数据权重为1，历史数据按λ^k衰减
    for (int i = 0; i < data->num; ++i) {
        float weight = powf(lambda, i); // 第i个数据（从最新到最旧）权重为λ^i
        float x = data->x[i];
        float y = data->y[i];
        
        sum_w += weight;
        sum_wx += weight * x;
        sum_wy += weight * y;
        sum_wx2 += weight * x * x;
        sum_wxy += weight * x * y;
    }
    // 改进的数值稳定性处理
    float denominator = sum_w * sum_wx2 - sum_wx * sum_wx;
    float threshold = 1e-6f * sum_w * sum_w; // 相对阈值，避免绝对阈值的局限性
    
    if (fabs(denominator) < threshold) {
        // 当分母接近零时，退化为加权平均
        data->a = 0.0f;
        data->b = sum_wy / sum_w;
    } else {
        data->a = (sum_w * sum_wxy - sum_wx * sum_wy) / denominator;
        data->b = (sum_wx2 * sum_wy - sum_wx * sum_wxy) / denominator;
    }
}
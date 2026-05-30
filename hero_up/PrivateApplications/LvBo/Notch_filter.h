// 定义滤波器结构体
typedef struct {
    // 滤波器系数
    float b0, b1, b2;
    float a1, a2;
    // 滤波器状态
    float x1, x2; // 前两个输入样本
    float y1, y2; // 前两个输出样本
} Biquad;
void initBandStopFilter(Biquad *filter, float fs, float notchFreq, float Q);
float processSample(Biquad *filter, float input);
// 双二阶滤波器结构体
typedef struct {
    float b0, b1, b2;    // 分子系数
    float a1, a2;        // 分母系数
    float x1, x2;        // 输入延迟单元
    float y1, y2;        // 输出延迟单元
} BiquadLPF;
void initLowPassFilter(BiquadLPF* filter, float fs, float fc, float Q);
float processLPF(BiquadLPF* filter, float input);
typedef struct {
    float prev_input;
    float prev_output;
} FirstOrderHighPassFilterState_t;
void HighPassFilter_Init(FirstOrderHighPassFilterState_t *state);
float HighPassFilter_Process(FirstOrderHighPassFilterState_t *state, float current_input);
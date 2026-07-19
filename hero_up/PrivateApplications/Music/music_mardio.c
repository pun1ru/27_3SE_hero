/**
 * @file music_mardio.c
 * @author 3SE 马丢 lzq
 * @brief 蜂鸣器音乐播放实现
 * @note  v2: +play_music_notes() 每音独立时长
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "music_mardio.h"

/*--- 休止符判定阈值 ---*/
#define REST_NOTE_MIN  100   /* note >= 100 视为休止 */

/**
 * @brief   播放一个音 (v1, 旧版)
 * @param   pr        预分频器值 (决定频率)
 * @param   wait_time 持续时间 (FreeRTOS ticks)
 * @param   htim      TIM 句柄
 */
void play_music(int pr, int wait_time, TIM_HandleTypeDef htim)
{
    if(pr > 0)
    {
        __HAL_TIM_PRESCALER(&htim, pr);
        __HAL_TIM_SetCompare(&htim, BUZZER_TIM_CHANNEL, 50);
        vTaskDelay(wait_time);
    }
    else
    {
        __HAL_TIM_PRESCALER(&htim, 0);
        __HAL_TIM_SetCompare(&htim, BUZZER_TIM_CHANNEL, 10);
        vTaskDelay(wait_time / 60);
    }
}

/**
 * @brief   将 mardio 音高编号转为预分频器值 (v1)
 * @param   note  音高编号 (62=中央C=261Hz)
 * @retval  pr    预分频器值, <=0 表示休止
 */
int from_notes_to_pr(float note)
{
    int pr;
    float hz;

    /* 十二平均律: 261 * 2^((note-62)/12) */
    hz = 261 * pow(2, (note - 62) / 12.0);
    pr = (int)168000000 / hz / 126000;

    if(note > 150)
        pr = -1;   /* 休止标记 */

    return pr;
}

/**
 * @brief   初始化蜂鸣器 TIM + PWM
 * @param   htim    TIM 句柄
 * @param   Channel TIM 通道
 * @retval  0 = 成功
 */
int music_init(TIM_HandleTypeDef htim, uint32_t Channel)
{
    HAL_TIM_Base_Start(&htim);
    HAL_TIM_PWM_Start(&htim, BUZZER_TIM_CHANNEL);
    HAL_TIM_PWM_Start(&htim, Channel);
    return 0;
}

/**
 * @brief   播放整首曲子 (v1, 固定 wait_time)
 * @param   notes     音高数组
 * @param   size      数组长度
 * @param   wait_time 每个音的固定时长 (FreeRTOS ticks)
 * @param   htim      TIM 句柄
 */
void all_paly_music(float *notes, int size, int wait_time, TIM_HandleTypeDef htim)
{
    for(int i = 0; i < size; i++)
    {
        play_music(from_notes_to_pr(notes[i]), wait_time, htim);
    }
}

/*===========================================================================
 *  v2 接口 (每音独立时长)
 *===========================================================================*/

/**
 * @brief   播放乐谱 (每音独立时长, v2)
 * @param   score   MusicNote 数组
 * @param   len     数组长度
 * @param   htim    TIM 句柄
 * @note    每个音符播放自己的 dur_ms, 延时通过 vTaskDelay(pdMS_TO_TICKS()) 实现
 *          休止符: note >= REST_NOTE_MIN 时关闭 PWM 输出
 */
void play_music_notes(const MusicNote* score, int len, TIM_HandleTypeDef htim)
{
    for(int i = 0; i < len; i++)
    {
        float   note   = score[i].note;
        uint16_t dur_ms = score[i].dur_ms;

        if(note < REST_NOTE_MIN)
        {
            /* 正常音符: 设频率, 开 PWM */
            int pr = from_notes_to_pr(note);
            if(pr > 0)
            {
                __HAL_TIM_PRESCALER(&htim, pr);
                __HAL_TIM_SetCompare(&htim, BUZZER_TIM_CHANNEL, 50);
            }
            else
            {
                /* 极低音或无效 → 静音 */
                __HAL_TIM_SetCompare(&htim, BUZZER_TIM_CHANNEL, 0);
            }
        }
        else
        {
            /* 休止符: 关闭输出 */
            __HAL_TIM_SetCompare(&htim, BUZZER_TIM_CHANNEL, 0);
        }

        /* 等待指定时长 (ms → FreeRTOS ticks) */
        vTaskDelay(pdMS_TO_TICKS(dur_ms));
    }
}

/*===========================================================================
 *  v2 用法示例:
 *
 *  // 1. 在 music_mardio.h 的 _MUSIC_KU 区定义乐谱:
 *  static const MusicNote my_song[] = {
 *      {16, 400},   // 音高16, 持续400ms
 *      {14, 200},
 *      {12, 200},
 *      {100, 150},  // 休止符150ms
 *      {17, 300},
 *  };
 *
 *  // 2. 播放:
 *  play_music_notes(my_song,
 *                    sizeof(my_song) / sizeof(my_song[0]),
 *                    htim4);
 *
 *  // MIDI 转换:
 *  //   python tools/midi_parser.py song.mid --channel 0 --format struct
 *  //   输出直接复制到上面的 _MUSIC_KU 区
 *===========================================================================*/

#include "LK_485_driver.h" // 假设这是您的头文件名
#include <string.h>          // 用于 memcpy

/*
=================================================================================
                            协议发送部分 (修改后)
=================================================================================
*/

/**
 * @brief       计算字节数组的校验和 (累加和，保留低8位)
 * @param[in]   data: 数据指针
 * @param[in]   len:  数据长度
 * @return      8位的校验和
 */
static uint8_t calculate_checksum(const uint8_t* data, uint16_t len)
{
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; ++i) {
        sum += data[i];
    }
    return sum;
}

/**
 * @brief       瓴控电机 - 转矩闭环控制 (RS485协议)
 * @param       motor_id:    电机ID (1-32)
 * @param       tx_buffer:   用于存放组装好的数据包的缓冲区
 * @param       iq_control:  转矩电流控制值 (-2048 到 2048)
 * @return      uint16_t:    组装好的数据包的总长度
 * @note        此函数已为瓴控RS485协议适配
 */
uint16_t LK_iqControl_485(uint8_t motor_id, uint8_t* tx_buffer, int16_t iq_control)
{
    const uint8_t cmd = 0xA1;
    const uint8_t data_len = sizeof(iq_control);

    tx_buffer[0] = 0x3E; // 帧头
    tx_buffer[1] = cmd;
    tx_buffer[2] = motor_id;
    tx_buffer[3] = data_len;
    tx_buffer[4] = calculate_checksum(tx_buffer, 4); // 命令校验和

    memcpy(&tx_buffer[5], &iq_control, data_len);
    tx_buffer[5 + data_len] = calculate_checksum(&tx_buffer[5], data_len); // 数据校验和

    return 5 + data_len + 1; // 总长度: 5(命令) + 2(数据) + 1(校验) = 8字节
}

/**
 * @brief       瓴控电机 - 速度闭环控制 (RS485协议)
 * @param       motor_id:      电机ID
 * @param       tx_buffer:     发送缓冲区
 * @param       speed_control: 速度控制值 (单位: 0.01dps/LSB)
 * @return      uint16_t:      数据包总长度
 */
uint16_t LK_speedControl_485(uint8_t motor_id, uint8_t* tx_buffer, int32_t speed_control)
{
    const uint8_t cmd = 0xA2;
    const uint8_t data_len = sizeof(speed_control);

    tx_buffer[0] = 0x3E;
    tx_buffer[1] = cmd;
    tx_buffer[2] = motor_id;
    tx_buffer[3] = data_len;
    tx_buffer[4] = calculate_checksum(tx_buffer, 4);

    memcpy(&tx_buffer[5], &speed_control, data_len);
    tx_buffer[5 + data_len] = calculate_checksum(&tx_buffer[5], data_len);

    return 5 + data_len + 1; // 总长度: 5 + 4 + 1 = 10字节
}

/**
 * @brief       瓴控电机 - 读取电机状态1 (温度,电压,错误) (RS485协议)
 * @param       motor_id:  电机ID
 * @param       tx_buffer: 发送缓冲区
 * @return      uint16_t:  数据包总长度
 */
uint16_t LK_stateRead_1_485(uint8_t motor_id, uint8_t* tx_buffer)
{
    const uint8_t cmd = 0x9A;
    const uint8_t data_len = 0; // 此命令无数据负载

    tx_buffer[0] = 0x3E;
    tx_buffer[1] = cmd;
    tx_buffer[2] = motor_id;
    tx_buffer[3] = data_len;
    tx_buffer[4] = calculate_checksum(tx_buffer, 4);

    return 5; // 总长度: 5字节
}
//最重要的mg
uint16_t LK_stateRead_2_485(uint8_t motor_id, uint8_t* tx_buffer)
{
    const uint8_t cmd = 0x9C;
    const uint8_t data_len = 0; // 此命令无数据负载

    tx_buffer[0] = 0x3E;
    tx_buffer[1] = cmd;
    tx_buffer[2] = motor_id;
    tx_buffer[3] = data_len;
    tx_buffer[4] = calculate_checksum(tx_buffer, 4);

    return 5; // 总长度: 5字节
}
//读温度，相电流什么的，没什么用
uint16_t LK_stateRead_3_485(uint8_t motor_id, uint8_t* tx_buffer)
{
    const uint8_t cmd = 0x9D;
    const uint8_t data_len = 0; // 此命令无数据负载

    tx_buffer[0] = 0x3E;
    tx_buffer[1] = cmd;
    tx_buffer[2] = motor_id;
    tx_buffer[3] = data_len;
    tx_buffer[4] = calculate_checksum(tx_buffer, 4);

    return 5; // 总长度: 5字节
}

// ... 其他发送函数和 calculate_checksum 函数保持不变 ...
/*
=================================================================================
                            协议接收部分 (新增)
=================================================================================
*/

/**
 * @brief       解析从瓴控电机返回的RS485数据包 (安全版本)
 * @param[in]   rx_buffer: 接收到的完整数据包的缓冲区指针
 * @param[in]   rx_len:    接收到的数据包的长度
 * @param[out]  state:     指向用于存储解析后数据的结构体指针
 * @return      int8_t:    解析结果
 *                       -  0: 解析成功
 *                       - -1: 校验和错误
 *                       - -2: 长度错误
 *                       - -3: 帧头错误或指针为空
 *                       - -4: 未知的命令码
 */
int8_t LK_RS485_Packet_Parse_485(const uint8_t* rx_buffer, uint16_t rx_len, LkMotor_State_t* state)
{
    // 1. 基本安全检查
    if (rx_buffer == NULL || state == NULL) {
        return -3; // 指针为空
    }
    // 检查最小帧长度
    if (rx_len < 5) {
        return -2; // 长度不足
    }
    // 检查帧头
    if (rx_buffer[0] != 0x3E) {
        return -3; // 帧头错误
    }

    // 2. 校验命令帧 (前5个字节)
    if (calculate_checksum(rx_buffer, 4) != rx_buffer[4]) {
        return -1; // 命令校验和错误
    }

    // 3. 提取数据长度，并校验数据帧 (如果存在)
    uint8_t data_len = rx_buffer[3];
    if (data_len > 0) {
        // 检查总长度是否匹配: 5字节命令帧 + N字节数据 + 1字节数据校验和
        if (rx_len != 5 + data_len + 1) {
            return -2; // 数据帧总长度不匹配
        }
        // 校验数据部分
        if (calculate_checksum(&rx_buffer[5], data_len) != rx_buffer[5 + data_len]) {
            return -1; // 数据校验和错误
        }
    } else {
        // 如果data_len为0，确保接收长度就是5
        if (rx_len != 5) {
            return -2; // 无数据负载时，长度应为5
        }
    }

    // 4. 所有校验通过，开始根据命令码解析数据
    uint8_t cmd = rx_buffer[1];
    const uint8_t* data_payload = &rx_buffer[5]; // 数据负载的起始地址

    // 只有在完全校验成功后才增加帧计数器
    state->frame_counter++; 

    switch (cmd) {
        case 0x9A: // 读取电机状态1 的回复
        case 0x9B: // 清除错误 的回复
        {
            if (data_len < 7) return -2; // 确保数据负载长度足够
            state->temperature = data_payload[0];
            memcpy(&state->voltage,     &data_payload[1], sizeof(state->voltage));
            memcpy(&state->current,     &data_payload[3], sizeof(state->current));
            state->motorState  = data_payload[5];
            state->errorState  = data_payload[6];
            break;
        }

        // 所有控制命令(A0-A8)和读取状态2(9C)的回复格式都相同
        case 0xA0: case 0xA1: case 0xA2: case 0xA3:
        case 0xA4: case 0xA5: case 0xA6: case 0xA7:
        case 0xA8: case 0x9C:
        {
            if (data_len < 7) return -2; // 确保数据负载长度足够
            state->temperature = data_payload[0];
            memcpy(&state->torque_current, &data_payload[1], sizeof(state->torque_current));
            memcpy(&state->speed,          &data_payload[3], sizeof(state->speed));
            memcpy(&state->encoder,        &data_payload[5], sizeof(state->encoder));
            break;
        }

        case 0x92: // 读取多圈角度 的回复
        {
            if (data_len < 8) return -2; // 确保数据负载长度足够
            memcpy(&state->multi_turn_angle, data_payload, sizeof(state->multi_turn_angle));
            break;
        }
        
        // 对于只返回相同命令的回复(如电机开关0x80, 0x88)，无需特殊处理数据
        case 0x80:
        case 0x88:
        case 0x81:
            if (data_len != 0) return -2; // 确保无数据负载
            break;

        default:
            // 其他未处理的命令回复
            return -4; // 返回一个特定的错误码表示未知命令
    }

    return 0; // 解析成功
}
//示例程序
//	/*485。。。uart发送*/
//		 // tx_buffer需要足够大以容纳最长的RS485帧
//	// 例如，多圈位置控制2的命令长度为 5 + 12 + 1 = 18字节
//	 static uint8_t tx_buffer[32]; //这个static不能关！
//	 /*局部变量的生命周期：局部变量的生命周期只在函数执行期间。函数一旦返回，它所占用的栈空间就会被认为“无效”，随时可能被其他函数或中断覆盖。
//		DMA的生命周期：DMA传输的生命周期是从 HAL_UART_Transmit_DMA 调用开始，到传输完成中断发生为止。这个时间远远长于函数的执行时间。
//		你把一个“短命”的局部变量的地址交给了“长寿”的DMA去处理，当函数返回后，这个地址就成了一个“悬空指针”，指向一块不安全的内存。这就是问题的根源！*/
//	// 假设的UART发送函数，你需要自己实现
//	// ...
//	uint16_t tx_len; // 用于存储数据包的实际长度
//	uint8_t pitch_motor_id = 1; // 假设电机ID是1
//	if(CONTROL_STOP != _robotState->ctrl_terminal)
//			// 调用修改后的函数，它会返回数据包的实际长度
//			tx_len = LK_iqControl_485(pitch_motor_id, tx_buffer, _gimbalControl->GimbalMotorControl.pitch_target_output);
//	else
//			tx_len = LK_iqControl_485(pitch_motor_id, tx_buffer, 0);
//	// 使用UART发送函数，发送实际长度的数据
//	tx_len = LK_Motor_on_485(pitch_motor_id,tx_buffer);
//	
//	/*避免DMA竞争状态，就是波特率限制了uart发送速度，只有HAL说好了才再次发送*/
//	//if(HAL_UART_GetState(&PITCH_UART) == HAL_UART_STATE_READY)
//	 if(uart10_tx_complete==1){
//		uart10_tx_complete=0;
//		HAL_UART_Transmit_DMA(&PITCH_UART,tx_buffer, tx_len);
//	 }
//	//HAL_UART_Transmit(&PITCH_UART, tx_buffer, tx_len, 100); // 换成这个阻塞式函数
//	 接收部分
//	if(huart == &PITCH_UART){
//		LK_RS485_Packet_Parse_485(uart10RecBuffer,Size,&lkmoto_State_t);
//		 HAL_UARTEx_ReceiveToIdle_DMA(&huart10, uart10RecBuffer, MAX_RECEIVE_BUFFER_LENGTH);
//	}

/**
 * @brief       瓴控电机 - 电机关闭命令 (RS485协议)
 * @param       motor_id:  电机ID
 * @param       tx_buffer: 发送缓冲区
 * @return      uint16_t:  数据包总长度
 */
uint16_t LK_Motor_off_485(uint8_t motor_id, uint8_t* tx_buffer)
{
    const uint8_t cmd = 0x80;
    const uint8_t data_len = 0;

    tx_buffer[0] = 0x3E;
    tx_buffer[1] = cmd;
    tx_buffer[2] = motor_id;
    tx_buffer[3] = data_len;
    tx_buffer[4] = calculate_checksum(tx_buffer, 4);

    return 5;
}
//打开点击
uint16_t LK_Motor_on_485(uint8_t motor_id, uint8_t* tx_buffer)
{
    const uint8_t cmd = 0x88;
    const uint8_t data_len = 0;

    tx_buffer[0] = 0x3E;
    tx_buffer[1] = cmd;
    tx_buffer[2] = motor_id;
    tx_buffer[3] = data_len;
    tx_buffer[4] = calculate_checksum(tx_buffer, 4);

    return 5;
}
//电机停止
uint16_t LK_Motor_stop_485(uint8_t motor_id, uint8_t* tx_buffer)
{
    const uint8_t cmd = 0x81;
    const uint8_t data_len = 0;

    tx_buffer[0] = 0x3E;
    tx_buffer[1] = cmd;
    tx_buffer[2] = motor_id;
    tx_buffer[3] = data_len;
    tx_buffer[4] = calculate_checksum(tx_buffer, 4);

    return 5;
}
// ... 您可以按照这个格式，继续修改和注释其他您需要用到的发送函数 ...你妈的不帮我写完
//附录：老版本的有问题的函数

/**
 * @brief       解析从瓴控电机返回的RS485数据包
 * @param[in]   rx_buffer: 接收到的完整数据包的缓冲区指针
 * @param[in]   rx_len:    接收到的数据包的长度
 * @param[out]  state:     指向用于存储解析后数据的结构体指针
 * @return      int8_t:    解析结果
 *                       -  0: 解析成功
 *                       - -1: 校验和错误
 *                       - -2: 长度错误
 *                       - -3: 帧头错误或指针为空
 */
//int8_t LK_RS485_Packet_Parse_485(const uint8_t* rx_buffer, uint16_t rx_len, LkMotor_State_t* state)
//{
//    // 1. 基本安全检查
//    if (rx_buffer == NULL || state == NULL || rx_len < 5) {
//        return -3; // 指针为空或长度不足
//    }
//    if (rx_buffer[0] != 0x3E) {
//        return -3; // 帧头错误
//    }

//    // 2. 校验命令帧 (前5个字节)
//    if (calculate_checksum(rx_buffer, 4) != rx_buffer[4]) {
//        return -1; // 命令校验和错误
//    }

//    // 3. 提取数据长度，并校验数据帧 (如果存在)
//    uint8_t data_len = rx_buffer[3];
//    if (data_len > 0) {
//        // 检查总长度是否匹配: 5字节命令帧 + N字节数据 + 1字节数据校验和
//        if (rx_len != 5 + data_len + 1) {
//            return -2; // 数据帧总长度不匹配
//        }
//        // 校验数据部分
//        if (calculate_checksum(&rx_buffer[5], data_len) != rx_buffer[5 + data_len]) {
//            return -1; // 数据校验和错误
//        }
//    }

//    // 4. 所有校验通过，开始根据命令码解析数据
//    uint8_t cmd = rx_buffer[1];
//    const uint8_t* data_payload = &rx_buffer[5]; // 数据负载的起始地址

//    state->frame_counter++; // 成功解析一帧，计数器+1

//    switch (cmd) {
//        case 0x9A: // 读取电机状态1 的回复
//        case 0x9B: // 清除错误 的回复
//        {
//            state->temperature = *(int8_t*)(data_payload + 0);
//            state->voltage = *(int16_t*)(data_payload + 1);
//            state->current = *(int16_t*)(data_payload + 3);
//            state->motorState = *(uint8_t*)(data_payload + 5);
//            state->errorState = *(uint8_t*)(data_payload + 6);
//            break;
//        }

//        // 所有控制命令(A0-A8)和读取状态2(9C)的回复格式都相同
//        case 0xA0:
//        case 0xA1: // 这是你之前CAN协议中对应的命令
//        case 0xA2:
//        case 0xA3:
//        case 0xA4:
//        case 0xA5:
//        case 0xA6:
//        case 0xA7:
//        case 0xA8:
//        case 0x9C:
//        {
//            // 注意！数据偏移量与你之前CAN协议的完全不同！
//            // 必须严格按照RS485协议文档来解析
//            state->temperature = *(int8_t*)(data_payload + 0);
//            state->torque_current = *(int16_t*)(data_payload + 1); // 对应你CAN协议里的 aData[3]<<8 | aData[2]
//            state->speed = *(int16_t*)(data_payload + 3);          // 对应你CAN协议里的 aData[5]<<8 | aData[4]
//            state->encoder = *(uint16_t*)(data_payload + 5);        // 对应你CAN协议里的 aData[7]<<8 | aData[6]
//            break;
//        }

//        case 0x92: // 读取多圈角度 的回复
//        {
//            state->multi_turn_angle = *(int64_t*)(data_payload + 0);
//            break;
//        }
//        
//        // 对于只返回相同命令的回复(如电机开关0x80, 0x88)，无需特殊处理数据
//        // 但解析函数依然会通过校验，返回0表示成功收到回复
//        case 0x80:
//        case 0x88:
//        case 0x81:
//            break;

//        default:
//            // 其他未处理的命令回复
//            break;
//    }

//    return 0; // 解析成功
//}
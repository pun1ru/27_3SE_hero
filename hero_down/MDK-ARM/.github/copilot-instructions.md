# HERO 2026 电控代码编程规范

基于《2024赛季电控一般兵种通用代码编程规范v1.1》，适用于 STM32H723 / FreeRTOS / HAL 项目。

## 代码风格

### 命名规范

| 类型 | 命名法 | 示例 |
|------|--------|------|
| 结构体名、函数名 | PascalCase（首单词大写） | `ChassisControl`, `ChassisInputUpdate()` |
| 全局变量 | camelCase（首单词小写） | `chassisControl` |
| 局部变量、函数形参 | snake_case（小写+下划线） | `chassis_control`, `rec_data` |
| 结构体成员变量 | snake_case | `follow_enable`, `speed_need_pid` |
| 宏定义 | UPPER_SNAKE_CASE | `#define CHASSIS_ON` |
| 源文件/头文件 | snake_case | `peripheral_receive_task.c` |
| 文件夹 | PascalCase | `PrivateDrivers`, `Tasks` |
| 指针变量 | 加 `ptr`/`handle` 后缀或前后加 `_` | `motor_ptr`, `_chassisControl` |

- **有物理意义的变量必须加单位后缀**（定义在 task.h 中）：
  - 角度 `_d`（度）、弧度 `_r`、角速度 `_rps`、转速 `_rpm`/`_rps`
  - 无单位后缀 = 该变量单位为 LSB
- for 循环计次允许用 `i`, `j`，否则禁止无意义命名

### 格式要求

- **Tab = 4 个空格，编码 UTF-8**（Keil: 工具栏小扳手 → Editor 分栏配置）
- **左花括号另起一行**（Allman 风格），同级缩进保持一致
- 操作符两侧加空格：`i = a * b;`
- 语义嵌套不超过 3 级 Tab 缩进，超过则重构或封装
- 状态量/标志位必须用宏定义或 `[0,1]`，禁止裸数字

### 注释规范

- 结构体定义、函数定义开头使用 Doxygen 注释：
```c
/**
 * @brief 遥操作指令接收任务，处理接收遥控器数据
 * @param[in]  rec_data  遥控器接收原始数据结构体
 * @param[in]  channel_num  拨杆通道标志位
 * @return 通道拨杆值 -660~660
 */
static int16_t DT7GetChannelVal(const DT7RecData* rec_data, uint8_t channel_num);
```

### include 顺序

分三层，空行隔开：
1. C 标准库：`#include <math.h>`
2. STM32/HAL 库：`#include "tim.h"`
3. 自编/第三方库：`#include "general_task_include.h"`

---

## 架构

### 顶层目录结构

```
GeneralHeader/        — 通用配置参数头文件（宏开关、方位检索、尺寸参数等）
PrivateApplications/  — 子应用（PID、算法等）
PrivateDrivers/       — 自编写驱动（CAN、DT7、IMU 等）
Tasks/                — 任务线程源文件
```

### 任务源文件划分（4 个）

| 源文件 | 职责 |
|--------|------|
| `initial_task.c` | 外设初始化、中断使能、PWM 开启，完成后创建所有线程并销毁自身 |
| `peripheral_receive_task.c` | 外设接收：IMU、CAN 中断回调、遥控器、裁判系统等 |
| `peripheral_transmit_task.c` | 外设发送：UI、电机控制帧发送函数定义（调用在 control 中） |
| `robot_control_task.c` | 控制相关：decision（目标赋值）+ control（闭环控制） |
| `state_task.c` | 状态机 + 线程状态监测（软件看门狗 + 硬件独立看门狗喂狗） |

- 每个任务线程用明显注释行隔开：`/*------ xxx task ------*/`
- 每个 task 的变量定义写在对应线程块开头，**禁止全部挤在源文件开头**
- 任务函数保持简洁，只调用几个函数，**禁止在任务函数中写详细计算**

### 遥控信号统一化

不同来源的遥控信号（DT7、图传链路、自定义控制器等）统一整合为标准遥控信号类型（定义在 `peripheral_receive_task.h` 中），其他 task 通过全局指针读取，无需包含各类驱动头文件。

---

## 头文件依赖规则（向下依赖）

- **`general_task_include.h`** 是唯一的通用包含头文件，包含所有 task 头文件、GeneralHeaders、C 标准库、FreeRTOS 等
- **其他头文件尽量减少 include**，只包含自身声明/定义所需的最少头文件
- 仅在 `.c` 中使用的头文件在 `.c` 中 include，**不要写在 `.h` 中**
- 原则：上层依赖下层，头文件不重复包含

---

## 全局变量规则

**总原则：变量在哪个源文件定义，就只能在该源文件中修改。其他源文件只能通过 `const` 指针读取。**

```c
// robot_control_task.c — 定义变量
ChassisControl chassisControl;
const ChassisControl* _chassisControl = &chassisControl;  // 全局常量指针

// general_task_include.h — 声明指针
extern const ChassisControl* _chassisControl;
```

- 只允许与任务直接相关的大结构体或重要标志位通过常量指针暴露
- 尽量减少全局变量数量，优先使用局部变量
- 全局变量尽可能塞到任务相关的主要结构体中

---

## 编程约定

### 函数设计

- 仅在一个源文件中调用的函数 → 加 `static` 限制作用域
- 参数为传址只读 → 加 `const` 修饰：`const ChassisControl* chassis`
- 函数形参优先采用传址（指针），便于在线程文件中定义变量后传入库函数读写
- 驱动/应用库：对外 API 函数声明在 `.h` 中，其他函数全部 `static`

---

## 编译与烧录

- 编译器：MDK-ARM (Keil)，目标芯片：STM32H723VGTx
- 烧录工具：pyocd + cmsis-dap
- 构建系统：CMSIS cbuild

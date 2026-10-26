/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Description        : CH32V003 智能温控风扇
 *                      SSD1306(128x32) + NTC热敏电阻 + 3个按键 + PWM风扇
 *
 *  硬件连接:
 *    PA1   OLED SDA      (软件 I2C, 与晶振脚共用 -> 见 OLED_Init 里的 PA12_RM 处理)
 *    PA2   OLED SCL      (换回 PC1/PC2 只需改下面 3 个宏, 见 OLED 软件 I2C 段)
 *    PD5   NTC 分压中点  (ADC_IN5, 10k 分压, 见 NTC_AT_VDD 说明)
 *    PC3   按键 "+"
 *    PC4   按键 "-"
 *    PD3   按键 "OK"
 *    PD2   风扇 PWM 输出 (TIM1_CH1, 约 20kHz)
 *    PC5   风扇 12V 电源 MOS (高电平导通; 见"风扇电源 (PC5 = 12V MOS)"段)
 *    PD4   风扇电源 LED 指示 (与 PC5 同步: 12V 导通 = 亮)
 *    PD6   运行模式 LED 指示 (手动 = 亮, 自动 = 灭)
 *    PC2   风扇 FG 转速输入 (EXTI2 下降沿计数; 外部 10k 上拉到 3.3V + 内部上拉, 见"风扇转速"段)
 *    PC0   调试串口 TX   (USART1 全重映射, 原 PD5 让给 NTC 使用)
 *
 *  界面 (128x32, 4 行 x 21 列, 6x8 点阵):
 *    各处位置由下方 X_* 宏决定: 整体 X_SHIFT, "RPM:" 组 X_FAN_SHIFT,
 *                                "Auto" 标题 X_AUTO_SHIFT, 手动转速行 X_MAN_ROW_SHIFT,
 *                                自动 Start/Stop 行 X_AUTO_ROW_SHIFT
 *    y0: T: 25.6C       RPM:12345      实时温度(左上) + 风扇实测转速(右侧, 由 PC2 的
 *                                       FG 信号测出; 检测不到信号时显示 "-----")
 *    y1: Manual    Auto                  当前运行模式 = 反白(白底黑字)芯片
 *    y2:              /  Start: 32.0C   自动=风扇启动温度 (手动此行留空)
 *    y3: Speed:  75%  /  Stop : 30.0C   手动=转速设定(画在此行) ; 自动=风扇停止温度
 *
 *  按键逻辑 (选中状态一律用反相高亮表示, 不再画 '>' 与 '*'):
 *    熄屏时    : 屏幕已关闭, 任意按键即可点亮(该次按键不再执行其它功能)
 *    双击 OK   : 只切换屏幕"常亮 <-> 自动熄屏"工作模式, 不改变当前亮灭
 *                (切到自动=重新开始 60s 计时到点才熄, 切到常亮=永不限时熄;
 *                OK 单击本就无功能, 与长按 OK 切模式互不干扰;
 *                右下角图标常显当前模式: Zz=自动熄屏, ON=常亮)
 *    长按 OK    : 切换 手动/自动 运行模式, 第二行反相芯片跟着切到对应标题上
 *    手动模式   : 短按(或长按/连发) +/- 直接改变风扇转速(0~100%, 步进 10%;
 *                 从 0% 启动时直接给启动阈值 20%, 减到阈值时直接停转)
 *    自动模式   : 短按 +/- 无反应;
 *                 长按 +/-     -> 进入设置: 第三行(启动温度)整行反相高亮,
 *                                 此时短按 +/- 改变温度(步进 1.0℃);
 *                 再长按 +/-   -> 反相高亮跳到第四行(停止温度), 短按 +/- 改温度;
 *                 再长按 +/-   -> 保存设置(立即写 Flash, 掉电不丢), 第四行恢复普通显示(第二行芯片仍在"自动"上)
 *
 *  自动(智能)模式控制 -- 按温差比例调速:
 *    进入自动模式时风扇处于停止状态;
 *    实时温度 > 启动温度 -> 启动风扇, 占空比 = 温差(℃) x 10%
 *        例: 启动温度 32.0℃, 当前 34.0℃ -> (34.0-32.0) x 10% = 20%
 *    之后每升高 1.0℃ 占空比 +10%, 到 100% 为止
 *        (即 启动温度 + 10.0℃ 时全速)
 *    温度下降时反向递减(PWM -10% / ℃), 直到 温度 <= 停止温度 -> 风扇停止
 *    运行过程中占空比不低于 FAN_MIN_DUTY(低速风扇会堵转)
 *    启停回差 TEMP_GAP = 3.0℃, 避免风扇在阈值附近频繁启停
 *    过温保护: 温度 >= 75.0℃(TEMP_FORCE_FULL) 时直接满速, 与设定阈值无关
 *
 *  开机画面:
 *    上电后 OLED 先显示开机画面(大字 "Smart Fan" 水平居中 + 下方进度条加载动画), 停留 BOOT_LOGO_MS(1.5s)
 *    清屏后才进入主界面; 期间风扇保持断电, 延时按 100ms 分片并喂狗
 *
 *  长期运行:
 *    看门狗    : IWDG 1.67s 超时, 主循环喂狗, 卡死自动复位
 *    熄屏/点亮 : 无按键 60s 后 OLED 0xAE 关显示(控温与 PWM 照常), 任意按键点亮;
 *                双击 OK 切换 常亮 / 自动熄屏(只改模式, 不立即熄屏)
 *    掉电保存  : 模式/手动转速/启停温度 改动后 3s 自动写 Flash(长按"保存"立即写),
 *                停电来电后 Cfg_Load() 恢复上次状态继续控温, 断电不丢配置;
 *                刷入固件会整片擦掉 Flash(含配置页) -> 刷完首次上电为 手动模式 + 0%
 *    风扇电源  : PC5 MOS 控 12V, 停转即断电; main() 第一条指令关断 MOS(消除上电
 *                误转); 合闸后先延时 2s(PWM 保持 0, 等 12V 稳定), 再从启动阈值 20%
 *                阶梯升速(软启动, 抑制启动浪涌打闪屏/MCU 复位)
 ********************************************************************************/

#include "debug.h"

/* ==================== OLED 软件 I2C ====================
 * 屏幕 SCL/SDA 接在哪个端口, 只改下面这 3 个宏:
 *   PA2=SCL / PA1=SDA : OLED_PORT GPIOA, OLED_PORT_RCC RCC_APB2Periph_GPIOA, OLED_OSC_PINS 1
 *   PC2=SCL / PC1=SDA : OLED_PORT GPIOC, OLED_PORT_RCC RCC_APB2Periph_GPIOC, OLED_OSC_PINS 0
 *
 * 为什么要 OLED_OSC_PINS: PA1/PA2 和外部晶振(OSC)脚是共用的。MounRiver 模板默认
 * SYSCLK_FREQ_48MHz_HSE, SystemInit() 会置位 AFIO_PCFR1.PA12_RM(bit15), 这时候
 * PA1/PA2 变成晶振输入脚, 当普通 GPIO 用完全没反应(屏幕全黑)。所以接 PA1/PA2 时:
 *   1) system_ch32v00x.c 里的系统时钟改用 HSI (本项目已改成 SYSCLK_FREQ_48MHZ_HSI);
 *   2) OLED_Init() 里清掉 PA12_RM (OLED_OSC_PINS = 1 时自动执行)。
 * 板上如果焊了 24MHz 晶振(PA1/PA2), 建议把屏幕接回 PC1/PC2: 晶振和它的负载电容
 * 会挂在 I2C 线上, 信号质量变差(而且此时用 PC 那套宏 OLED_OSC_PINS 要设 0)。
 * ==================================================== */
#define OLED_PORT        GPIOA
#define OLED_PORT_RCC    RCC_APB2Periph_GPIOA
#define OLED_OSC_PINS    1              /* 1 = SCL/SDA 接在 PA1/PA2(晶振脚)上 */
#define OLED_SCL_PIN     GPIO_Pin_2
#define OLED_SDA_PIN     GPIO_Pin_1
#define OLED_I2C_ADDR    0x78

#define OLED_SCLK_Clr()  GPIO_ResetBits(OLED_PORT, OLED_SCL_PIN)
#define OLED_SCLK_Set()  GPIO_SetBits(OLED_PORT, OLED_SCL_PIN)
#define OLED_SDIN_Clr()  GPIO_ResetBits(OLED_PORT, OLED_SDA_PIN)
#define OLED_SDIN_Set()  GPIO_SetBits(OLED_PORT, OLED_SDA_PIN)
#define IIC_READ_SDA     GPIO_ReadInputDataBit(OLED_PORT, OLED_SDA_PIN)

/* ==================== 按键 (按下为低电平) ==================== */
#define KEY_PLUS_PORT   GPIOC
#define KEY_PLUS_PIN    GPIO_Pin_3      /* "+"  */
#define KEY_MINUS_PORT  GPIOC
#define KEY_MINUS_PIN   GPIO_Pin_4      /* "-"  */
#define KEY_OK_PORT     GPIOD
#define KEY_OK_PIN      GPIO_Pin_3      /* "OK" */

/* ==================== 风扇 PWM (PD2 = TIM1_CH1) ==================== */
#define FAN_PWM_ARR     255             /* PWM 分辨率 8bit */
#define FAN_PWM_FREQ    20000           /* 目标 PWM 频率 20kHz (四线风扇标准) */

/* ==================== 风扇转速 (PC2 = FG 测速输入) ====================
 *  FG = 风扇的转速反馈方波(集电极开路输出):
 *      硬件: 外部 10k 上拉到 3.3V, 代码里再开内部上拉(GPIO_Mode_IPU),
 *            两者并联(约 9k), 上升沿更快, 不冲突
 *      每转输出 FAN_FG_PPR 个脉冲, 常见风扇是 2 个/转
 *  测速: PC2 下降沿触发中断累加脉冲, 主循环攒够 FAN_RPM_WINDOW_MS 换算一次:
 *      rpm = 脉冲数 / 每转脉冲数 x 60000 / 本窗口实际毫秒数
 *  (窗口毫秒数由 TIM2 的 1kHz 时基量出, 刷屏多花的时间不会让转速偏)
 * ==================================================================== */
#define FAN_FG_PORT         GPIOC
#define FAN_FG_PIN          GPIO_Pin_2      /* 风扇 FG 信号输入 */
#define FAN_FG_PPR          2               /* 每转脉冲数: 读数正好差一倍时改这里(1 或 4) */
#define FAN_RPM_WINDOW_MS   1000            /* 测速窗口(ms): 满这个时长才换算一次;
                                               调大 = 读数更稳更准但刷新更慢 */
#define FAN_RPM_FILTER_DIV  4               /* 转速一阶滤波: 新值权重 1/4; 1 = 不滤波 */
#define FAN_RPM_MAX         99999           /* 显示上限(第一行放得下 5 位) */

/* ==================== NTC 采样 (PD5 = ADC_IN5) ==================== */
#define NTC_PORT        GPIOD
#define NTC_PIN         GPIO_Pin_5
#define NTC_ADC_CH      ADC_Channel_5   /* CH32V003: PD5 = AIN5 */
#define NTC_R_SERIES    10000           /* 分压电阻 10kΩ */
#define NTC_ADC_FULL    1023            /* 10bit 满量程 */
#define NTC_AT_VDD      0               /* 1: VDD-NTC-ADC-R-GND (温度升高 ADC值升高)
                                           0: VDD-R-ADC-NTC-GND (温度升高 ADC值降低)
                                           实测接法为 NTC 下端接地: 升温时读数下降 -> 0 */
#define NTC_TAB_SIZE    30              /* 换算表点数: -20℃ ~ +125℃, 每 5℃ 一点 */

/* ==================== 6x8 ASCII 字库 (沿用原工程, 索引 = 字符-' ') ==================== */
const uint8_t Font6x8[] = {
    0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x2F,0x00,0x00,0x00,
    0x00,0x07,0x00,0x07,0x00,0x00, 0x14,0x7F,0x14,0x7F,0x14,0x00,
    0x24,0x2A,0x7F,0x2A,0x12,0x00, 0x23,0x13,0x08,0x64,0x62,0x00,
    0x36,0x49,0x55,0x22,0x50,0x00, 0x00,0x05,0x03,0x00,0x00,0x00,
    0x00,0x1C,0x22,0x41,0x00,0x00, 0x00,0x41,0x22,0x1C,0x00,0x00,
    0x14,0x08,0x3E,0x08,0x14,0x00, 0x08,0x08,0x3E,0x08,0x08,0x00,
    0x00,0x50,0x30,0x00,0x00,0x00, 0x08,0x08,0x08,0x08,0x08,0x00,
    0x00,0x60,0x60,0x00,0x00,0x00, 0x20,0x10,0x08,0x04,0x02,0x00,
    0x3E,0x51,0x49,0x45,0x3E,0x00, 0x00,0x42,0x7F,0x40,0x00,0x00,
    0x42,0x61,0x51,0x49,0x46,0x00, 0x21,0x41,0x45,0x4B,0x31,0x00,
    0x18,0x14,0x12,0x7F,0x10,0x00, 0x27,0x45,0x45,0x45,0x39,0x00,
    0x3C,0x4A,0x49,0x49,0x30,0x00, 0x01,0x71,0x09,0x05,0x03,0x00,
    0x36,0x49,0x49,0x49,0x36,0x00, 0x06,0x49,0x49,0x29,0x1E,0x00,
    0x00,0x36,0x36,0x00,0x00,0x00, 0x00,0x56,0x36,0x00,0x00,0x00,
    0x08,0x14,0x22,0x41,0x00,0x00, 0x14,0x14,0x14,0x14,0x14,0x00,
    0x00,0x41,0x22,0x14,0x08,0x00, 0x02,0x01,0x51,0x09,0x06,0x00,
    0x32,0x49,0x79,0x41,0x3E,0x00, 0x7E,0x11,0x11,0x11,0x7E,0x00,
    0x7F,0x49,0x49,0x49,0x36,0x00, 0x3E,0x41,0x41,0x41,0x22,0x00,
    0x7F,0x41,0x41,0x22,0x1C,0x00, 0x7F,0x49,0x49,0x49,0x41,0x00,
    0x7F,0x09,0x09,0x09,0x01,0x00, 0x3E,0x41,0x49,0x49,0x7A,0x00,
    0x7F,0x08,0x08,0x08,0x7F,0x00, 0x00,0x41,0x7F,0x41,0x00,0x00,
    0x20,0x40,0x41,0x3F,0x01,0x00, 0x7F,0x08,0x14,0x22,0x41,0x00,
    0x7F,0x40,0x40,0x40,0x40,0x00, 0x7F,0x02,0x0C,0x02,0x7F,0x00,
    0x7F,0x04,0x08,0x10,0x7F,0x00, 0x3E,0x41,0x41,0x41,0x3E,0x00,
    0x7F,0x09,0x09,0x09,0x06,0x00, 0x3E,0x41,0x51,0x21,0x5E,0x00,
    0x7F,0x09,0x19,0x29,0x46,0x00, 0x46,0x49,0x49,0x49,0x31,0x00,
    0x01,0x01,0x7F,0x01,0x01,0x00, 0x3F,0x40,0x40,0x40,0x3F,0x00,
    0x1F,0x20,0x40,0x20,0x1F,0x00, 0x3F,0x40,0x38,0x40,0x3F,0x00,
    0x63,0x14,0x08,0x14,0x63,0x00, 0x07,0x08,0x70,0x08,0x07,0x00,
    0x61,0x51,0x49,0x45,0x43,0x00,
    0x00,0x7F,0x41,0x41,0x00,0x00, 0x02,0x04,0x08,0x10,0x20,0x00,
    0x00,0x41,0x41,0x7F,0x00,0x00, 0x04,0x02,0x01,0x02,0x04,0x00,
    0x40,0x40,0x40,0x40,0x40,0x00, 0x00,0x01,0x02,0x04,0x00,0x00,
    0x20,0x54,0x54,0x54,0x78,0x00, 0x7F,0x48,0x44,0x44,0x38,0x00,
    0x38,0x44,0x44,0x44,0x20,0x00, 0x38,0x44,0x44,0x48,0x7F,0x00,
    0x38,0x54,0x54,0x54,0x18,0x00, 0x08,0x7E,0x09,0x01,0x02,0x00,
    0x0C,0x52,0x52,0x52,0x3E,0x00, 0x7F,0x08,0x04,0x04,0x78,0x00,
    0x00,0x44,0x7D,0x40,0x00,0x00, 0x20,0x40,0x44,0x3D,0x00,0x00,
    0x7F,0x10,0x28,0x44,0x00,0x00, 0x00,0x41,0x7F,0x40,0x00,0x00,
    0x7C,0x04,0x18,0x04,0x78,0x00, 0x7C,0x08,0x04,0x04,0x78,0x00,
    0x38,0x44,0x44,0x44,0x38,0x00, 0x7C,0x14,0x14,0x14,0x08,0x00,
    0x08,0x14,0x14,0x18,0x7C,0x00, 0x7C,0x08,0x04,0x04,0x08,0x00,
    0x48,0x54,0x54,0x54,0x20,0x00, 0x04,0x3F,0x44,0x40,0x20,0x00,
    0x3C,0x40,0x40,0x20,0x7C,0x00, 0x1C,0x20,0x40,0x20,0x1C,0x00,
    0x3C,0x40,0x30,0x40,0x3C,0x00, 0x44,0x28,0x10,0x28,0x44,0x00,
    0x0C,0x50,0x50,0x50,0x3C,0x00, 0x44,0x64,0x54,0x4C,0x44,0x00,
    0x00,0x08,0x36,0x41,0x00,0x00, 0x00,0x00,0x7F,0x00,0x00,0x00,
    0x00,0x41,0x36,0x08,0x00,0x00, 0x08,0x04,0x08,0x10,0x08,0x00,
};
/* ==================== I2C 底层驱动 ==================== */
void IIC_delay(void) { uint8_t t = 50; while(t--); }
void I2C_SDA_Out(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = OLED_SDA_PIN; GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; GPIO_Init(OLED_PORT, &GPIO_InitStructure);
}
void I2C_SDA_In(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = OLED_SDA_PIN; GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; GPIO_Init(OLED_PORT, &GPIO_InitStructure);
}
void I2C_SCL_Out(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = OLED_SCL_PIN; GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; GPIO_Init(OLED_PORT, &GPIO_InitStructure);
}
void I2C_Start(void) {
    I2C_SDA_Out(); OLED_SDIN_Set(); I2C_SCL_Out(); OLED_SCLK_Set(); IIC_delay();
    OLED_SDIN_Clr(); IIC_delay(); OLED_SCLK_Clr(); IIC_delay();
}
void I2C_Stop(void) {
    I2C_SDA_Out(); OLED_SDIN_Clr(); I2C_SCL_Out(); OLED_SCLK_Clr(); IIC_delay();
    OLED_SCLK_Set(); IIC_delay(); I2C_SDA_In(); IIC_delay();
}
void I2C_SendByte(uint8_t dat) {
    uint8_t i; I2C_SCL_Out(); I2C_SDA_Out();
    for(i = 0; i < 8; i++) {
        if(dat & 0x80) OLED_SDIN_Set(); else OLED_SDIN_Clr();
        dat <<= 1; IIC_delay(); OLED_SCLK_Set(); IIC_delay(); OLED_SCLK_Clr(); IIC_delay();
    }
    I2C_SDA_In(); IIC_delay(); OLED_SCLK_Set(); IIC_delay(); OLED_SCLK_Clr(); IIC_delay(); I2C_SDA_Out();
}

/* ==================== OLED 基本操作 ==================== */
void OLED_WrCmd(uint8_t cmd) {
    I2C_Start(); I2C_SendByte(OLED_I2C_ADDR); I2C_SendByte(0x00); I2C_SendByte(cmd); I2C_Stop();
}
void OLED_WrDat(uint8_t dat) {
    I2C_Start(); I2C_SendByte(OLED_I2C_ADDR); I2C_SendByte(0x40); I2C_SendByte(dat); I2C_Stop();
}
/* 连续写多个数据字节: 一次 Start/Stop, 比逐字节 OLED_WrDat 快 2 倍以上 */
void OLED_WrDataN(const uint8_t *dat, uint8_t len) {
    uint8_t i;
    I2C_Start(); I2C_SendByte(OLED_I2C_ADDR); I2C_SendByte(0x40);
    for(i = 0; i < len; i++) I2C_SendByte(dat[i]);
    I2C_Stop();
}
void OLED_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};

#if OLED_OSC_PINS
    /* PA1/PA2 是晶振(OSC)共用脚: MounRiver 模板默认 SYSCLK_FREQ_48MHz_HSE 时,
       SystemInit() 会把 AFIO_PCFR1 的 PA12_RM(bit15) 置 1, PA1/PA2 就变成晶振输入,
       代码里怎么翻转电平都不动 -> 屏幕全黑。这里清 0, 让 PA1/PA2 恢复成普通 GPIO。
       (前提: 系统时钟不能用 HSE, 见 system_ch32v00x.c 已改为 SYSCLK_FREQ_48MHZ_HSI) */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    AFIO->PCFR1 &= ~(1 << 15);
#endif

    RCC_APB2PeriphClockCmd(OLED_PORT_RCC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = OLED_SDA_PIN | OLED_SCL_PIN; GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; GPIO_Init(OLED_PORT, &GPIO_InitStructure);
    OLED_SCLK_Set(); OLED_SDIN_Set(); Delay_Ms(100);
    OLED_WrCmd(0xAE); OLED_WrCmd(0xD5); OLED_WrCmd(0x80); OLED_WrCmd(0xA8); OLED_WrCmd(0x1F);
    OLED_WrCmd(0xD3); OLED_WrCmd(0x00); OLED_WrCmd(0x40); OLED_WrCmd(0x8D); OLED_WrCmd(0x14);
    /* 0xA0 + 0xC0 = 关闭 SEG/COM 重映射, 屏幕整体旋转 180° (原为 0xA1 + 0xC8) */
    OLED_WrCmd(0x20); OLED_WrCmd(0x00); OLED_WrCmd(0xA0); OLED_WrCmd(0xC0); OLED_WrCmd(0xDA);
    OLED_WrCmd(0x02); OLED_WrCmd(0x81); OLED_WrCmd(0x8F); OLED_WrCmd(0xD9); OLED_WrCmd(0xF1);
    OLED_WrCmd(0xDB); OLED_WrCmd(0x40); OLED_WrCmd(0xA4); OLED_WrCmd(0xA6); OLED_WrCmd(0xAF);
}
void OLED_SetPos(uint8_t x, uint8_t y) {
    OLED_WrCmd(0xB0 + y); OLED_WrCmd(((x & 0xF0) >> 4) | 0x10); OLED_WrCmd(x & 0x0F);
}
void OLED_Clear(void) {
    uint8_t i, n;
    for(i = 0; i < 4; i++) {
        OLED_WrCmd(0xB0 + i); OLED_WrCmd(0x00); OLED_WrCmd(0x10);
        for(n = 0; n < 128; n++) OLED_WrDat(0x00);
    }
}
void OLED_DisplayOn(void)  { OLED_WrCmd(0xAF); }          /* 恢复显示(显存保留) */
void OLED_DisplayOff(void) { OLED_WrCmd(0xAE); }          /* 关闭显示, GDDRAM 保留 */

/* ---------------- 反相高亮: 显示层状态 ----------------
 *  g_rowHl = 当前"整行反相(白底黑字)"高亮的行号, SYM_NONE = 无高亮。
 *  OLED_ShowChar 发现目标行 == g_rowHl 时自动把字模取反, 因此
 *  ShowString / ShowNum / ShowTemp10 等所有上层绘制都会自动反相,
 *  配合 OLED_WrFill 铺的白色底条, 就用"整行反相"代替了原来的 '>' 光标与 '*'。
 *  (行号 Y_ROW1/Y_ROW2 在文件下方才定义, 这里只保存运行期比较用的值)
 * --------------------------------------------------------------- */
#define SYM_NONE    0xFF
static uint8_t g_rowHl = SYM_NONE;      /* 当前反相高亮的行 */

/* 字符写入: inv = 1 -> 字模取反(白底黑字) */
static void OLED_PutChar(uint8_t x, uint8_t y, char ch, uint8_t inv) {
    uint8_t c = (uint8_t)(ch - ' ');
    uint8_t i, buf[6];

    if (c > 94) c = 0;
    if (inv) {
        for (i = 0; i < 6; i++) buf[i] = (uint8_t)(~Font6x8[c * 6 + i]);
        OLED_SetPos(x, y);
        OLED_WrDataN(buf, 6);
        return;
    }
    OLED_SetPos(x, y);
    OLED_WrDataN(&Font6x8[c * 6], 6);
}
void OLED_ShowChar(uint8_t x, uint8_t y, char ch) {
    OLED_PutChar(x, y, ch, (uint8_t)(y == g_rowHl));    /* 高亮行自动反相 */
}
void OLED_ShowCharInv(uint8_t x, uint8_t y, char ch) {
    OLED_PutChar(x, y, ch, 1);                          /* 强制反相(第二行高亮条) */
}
void OLED_ShowString(uint8_t x, uint8_t y, char *str) {
    while(*str) { if (y > 3) break; OLED_ShowChar(x, y, *str++); x += 6; if(x > 122) { x = 0; y++; } }
}
void OLED_ShowNum(uint8_t x, uint8_t y, int32_t num, uint8_t len) {
    uint8_t i; uint8_t flag = 0; uint32_t abs_num; uint32_t temp = 1;
    if (num < 0) { OLED_ShowChar(x, y, '-'); abs_num = (uint32_t)(-num); x += 6; len--; }
    else { abs_num = (uint32_t)num; }
    for(i = 0; i < len; i++) temp *= 10; temp /= 10;
    for(i = 0; i < len; i++) {
        if (abs_num >= temp) flag = 1;
        if (flag || i == len - 1) OLED_ShowChar(x + i * 6, y, (abs_num / temp) % 10 + '0');
        else OLED_ShowChar(x + i * 6, y, ' ');
        temp /= 10;
    }
}

/* ---------------- 反相显示 (白底黑字) ---------------- */
/* 在 (x,y) 起写 w 个像素的纯色带(整页高度): dat=0xFF 点亮 / 0x00 熄灭 */
void OLED_WrFill(uint8_t x, uint8_t y, uint8_t w, uint8_t dat) {
    uint8_t i, buf[6];

    for (i = 0; i < 6; i++) buf[i] = dat;
    if ((uint16_t)x + w > 128) w = (uint8_t)(128 - x);          /* 限幅到右边缘 */
    while (w >= 6) { OLED_SetPos(x, y); OLED_WrDataN(buf, 6); x += 6; w -= 6; }
    if (w)         { OLED_SetPos(x, y); OLED_WrDataN(buf, w); }
}
void OLED_ShowStringInv(uint8_t x, uint8_t y, char *str) {
    while(*str) { OLED_ShowCharInv(x, y, *str++); x += 6; if(x > 122) break; }
}

/* ---------------- 大字显示 (6x8 字模 2x 放大 -> 12x16) ----------------
 * 用于开机画面: 每个源像素放大为 2x2(源行 r -> 输出行 2r、2r+1)。
 * 字符高 16px = 占两页: 先写 yPage 页(源行 0..3), 再写 yPage+1 页(源行 4..7),
 * 每页内逐列连续写, 每字符只需 2 次 SetPos。不参与主界面的行反相高亮逻辑。 */
static void OLED_PutCharBig(uint8_t x, uint8_t yPage, char ch) {
    uint8_t c = (uint8_t)(ch - ' ');
    uint8_t col, r, k;
    uint8_t src, p0, p1;
    uint8_t buf0[12], buf1[12];

    if (c > 94) c = 0;
    for (col = 0; col < 6; col++) {
        src = Font6x8[c * 6 + col];
        p0 = 0; p1 = 0;
        for (r = 0; r < 8; r++) {
            if (src & (1 << r)) {
                uint8_t bits = (uint8_t)(0x03u << ((r & 3) * 2));   /* 1 行 -> 2 行 */
                if (r < 4) p0 |= bits;                              /* 上半页 */
                else       p1 |= bits;                              /* 下半页 */
            }
        }
        for (k = 0; k < 2; k++) {                                   /* 水平 2x: 每列写两遍 */
            buf0[col * 2 + k] = p0;
            buf1[col * 2 + k] = p1;
        }
    }
    OLED_SetPos(x, yPage);     OLED_WrDataN(buf0, 12);
    OLED_SetPos(x, yPage + 1); OLED_WrDataN(buf1, 12);
}
void OLED_ShowStringBig(uint8_t x, uint8_t yPage, char *str) {
    while(*str) { OLED_PutCharBig(x, yPage, *str++); x += 12; }
}

/* ==================== 开机画面专用显存缓冲区 (128x32 = 512B) ====================
 * SSD1306 只能写不能读显存，进度条边框+填充需要读-改-写，故开辟本地缓冲区。
 * 所有绘图在缓冲区内完成，最后一次性 flush 到 OLED。splash 结束后缓冲区可被覆盖。 */
static uint8_t g_splashBuf[128 * 4];   /* 4 页 x 128 列 */

static void Splash_Pixel(uint8_t x, uint8_t y, uint8_t on) {
    if (x >= 128 || y >= 32) return;
    uint8_t *p = &g_splashBuf[(y >> 3) * 128 + x];
    if (on)  *p |=  (uint8_t)(1u << (y & 7));
    else     *p &= (uint8_t)(~(1u << (y & 7)));
}

static void Splash_HLine(uint8_t x0, uint8_t x1, uint8_t y, uint8_t on) {
    uint8_t x;
    for (x = x0; x <= x1; x++) Splash_Pixel(x, y, on);
}

static void Splash_VLine(uint8_t y0, uint8_t y1, uint8_t x, uint8_t on) {
    uint8_t y;
    for (y = y0; y <= y1; y++) Splash_Pixel(x, y, on);
}

static void Splash_Rect(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t on) {
    Splash_HLine(x0, x1, y0, on);
    Splash_HLine(x0, x1, y1, on);
    Splash_VLine(y0, y1, x0, on);
    Splash_VLine(y0, y1, x1, on);
}

static void Splash_FillRect(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t on) {
    uint8_t y;
    for (y = y0; y <= y1; y++) Splash_HLine(x0, x1, y, on);
}

/* 缓冲区版大字 (6x8 字模 2x 放大 -> 12x16)，写入 g_splashBuf */
static void Splash_PutCharBig(uint8_t x, uint8_t y, char ch) {
    uint8_t c = (uint8_t)(ch - ' ');
    uint8_t col, r, k;
    uint8_t src;
    if (c > 94) c = 0;
    for (col = 0; col < 6; col++) {
        src = Font6x8[c * 6 + col];
        for (r = 0; r < 8; r++) {
            if (src & (1u << r)) {
                for (k = 0; k < 2; k++) {
                    Splash_Pixel(x + col * 2 + k, y + r * 2,     1);
                    Splash_Pixel(x + col * 2 + k, y + r * 2 + 1, 1);
                }
            }
        }
    }
}

static void Splash_Flush(void) {
    uint8_t page;
    for (page = 0; page < 4; page++) {
        OLED_SetPos(0, page);
        OLED_WrDataN(&g_splashBuf[page * 128], 128);
    }
}

/* ==================== NTC 温度换算表 ====================
 *   B = 3950, R25 = 10kΩ
 *   ntcTemp10[] : 温度 x10 (℃)          -20.0 ~ +125.0℃, 步长 5℃
 *   ntcRes10[]  : NTC 阻值 (单位 10Ω)   与上面温度一一对应
 *   温度升高 -> 阻值减小, 查表后线性插值即可得到 0.1℃ 精度
 * ======================================================== */
static const int16_t  ntcTemp10[NTC_TAB_SIZE] = {
      -200,  -150,  -100,   -50,     0,    50,   100,   150,   200,   250,
       300,   350,   400,   450,   500,   550,   600,   650,   700,   750,
       800,   850,   900,   950,  1000,  1050,  1100,  1150,  1200,  1250,
};
static const uint16_t ntcRes10[NTC_TAB_SIZE] = {
     10538,  7790,  5825,  4403,  3362,  2592,  2017,  1584,  1254,  1000,
       804,   651,   530,   435,   359,   298,   249,   209,   176,   149,
       127,   109,    93,    81,    70,    61,    53,    46,    41,    36,
};
/* ==================== 参数配置 ==================== */
#define MODE_MANUAL         0
#define MODE_AUTO           1

/* 设置流程状态: 决定反相高亮停在第三/四行的哪一行 */
#define ST_NORMAL           0       /* 第二行: 模式标题上(手动或自动) */
#define ST_SET_START        1       /* 第三行: 风扇启动温度 */
#define ST_SET_STOP         2       /* 第四行: 风扇停止温度 */

#define KEY_NUM             3
#define KEY_PLUS            0
#define KEY_MINUS           1
#define KEY_OK              2

#define KEY_DEBOUNCE_TICKS   8      /* 消抖: 连续 8 次扫描电平一致 (约 8~10ms) */
#define KEY_LONG_PRESS_TICKS 600    /* 按住 0.6s 判定为长按 */
#define KEY_REPEAT_PERIOD    60     /* 长按之后的连发间隔(仅供手动调速使用) */

/* Key_Scan() 返回值 */
#define KEY_EV_CLICK        0x0007  /* bit0~2: 短按 (松手时判定) */
#define KEY_EV_LONG         0x0038  /* bit3~5: 长按 (按住到时间触发一次) */
#define KEY_EV_REPEAT       0x01C0  /* bit6~8: 长按之后的连发 */

#define TEMP_PERIOD_TICKS   250     /* 温度采样/调速周期(主循环次数, 约 0.3~0.6s) */

#define MANUAL_STEP         10      /* 手动调速步进 10% */
#define TEMP_STEP           10      /* 温度设置步进 1.0℃ (单位 0.1℃) */
#define TEMP_START_MIN      100     /* 启动温度下限 10.0℃ */
#define TEMP_START_MAX      600     /* 启动温度上限 60.0℃ */
#define TEMP_STOP_MIN       0       /* 停止温度下限  0.0℃ */
#define TEMP_STOP_MAX       550     /* 停止温度上限 55.0℃ */
#define TEMP_GAP            20      /* 启停回差 2.0℃ (太小会导致风扇频繁启停) */

#define FAN_START_DUTY      20      /* PWM 风扇启动阈值 %: 多数风扇低于此占空比不转,
                                       停转->启动时直接给到此值 */
#define FAN_MIN_DUTY        FAN_START_DUTY  /* 风扇最低启动占空比(防堵转), 0 = 完全按公式 */
#define AUTO_SLOPE_PER_DEG  10      /* 自动模式: 每升高 1.0℃ 增加的占空比 % (温差x10%) */
#define AUTO_DUTY_MAX       100     /* 自动模式占空比上限 % */
#define TEMP_FORCE_FULL     750     /* >= 75.0℃ 强制全速(过温保护, 与设定阈值无关) */
#define FAN_ON_SENSOR_ERR   0       /* NTC 异常时自动模式: 1=启动 0=停止 */
#define NTC_TEMP_ERR        ((int16_t)0x7FFF)   /* 传感器异常标志 */
#define NTC_FILTER_DIV      8       /* 一阶滤波: 新采样权重 1/8 */

/* ---- 长期运行相关 ---- */
#define SCREEN_OFF_SEC      60      /* 无按键操作 60s 后关屏(只关显示, 控温与 PWM 照常) */
#define DOUBLE_CLICK_MS     400     /* OK 双击间隔上限(ms): 两次单击间隔小于它算双击 */
/* 看门狗: IWDG 时钟源是 LSI(≈40kHz), 与主频无关;
 * 分频 64 -> 625Hz, 1044 计数 ≈ 1.67s */
#define IWDG_PRESCALE       IWDG_Prescaler_64     /* LSI / 64 = 625Hz */
#define IWDG_RELOAD         1044                  /* 1044 / 625Hz = 1.67s */

/* ==================== 界面坐标 (6x8 点阵, 每字符宽 6 像素) ====================
 *  位移量: 整体右移 X_SHIFT, 便于避让左侧外壳/排版留白
 *          X_FAN_SHIFT  只作用于第一行右侧的 "RPM:" 组
 *          X_AUTO_SHIFT 只作用于第二行 "Auto" 标题(含其反相芯片)
 *          X_MAN_ROW_SHIFT 只作用于手动模式的转速行(该行画在第四行)
 *          X_AUTO_ROW_SHIFT 只作用于自动模式的第三/四行(Start / Stop)
 * ============================================================================ */
#define X_SHIFT         7          /* 整体内容右移 7 像素   */
#define X_FAN_SHIFT     5           /* 第一行右侧 RPM 组再右移 5 像素 */
#define X_AUTO_SHIFT    15          /* 第二行 "Auto" 标题再右移 15 像素 */

#define Y_TEMP          0
#define Y_MODE          1           /* 第二行: 模式 */
#define Y_ROW1          2           /* 第三行: 启动温度(自动) ; 手动此行留空 */
#define Y_ROW2          3           /* 第四行: 停止温度(自动) / 转速(手动) */

#define X_TEMP_LBL      (X_SHIFT + 0)                   /* y0 "T:" 标签 */
#define X_TEMP          (X_SHIFT + 12)                  /* y0 温度数值 (紧跟 "T:") */
#define X_RPM_LBL_Y0    (X_SHIFT + X_FAN_SHIFT + 54)    /* y0 "RPM:" 标签 (4 字符) */
#define X_RPM_VAL_Y0    (X_SHIFT + X_FAN_SHIFT + 78)    /* y0 转速数值 (5 位, 右对齐) */

#define X_MODE_MAN      (X_SHIFT + 6)                   /* "Manual" 标题 */
#define X_MODE_AUTO     (X_SHIFT + X_AUTO_SHIFT + 54)   /* "Auto"   标题 */

#define X_ROW_LABEL     (X_SHIFT + 6)                   /* 第三/四行 标签 (7字符宽) */
#define X_ROW_VAL       (X_SHIFT + 48)                  /* 第三/四行 数值 */

/* 手动模式的转速行改画到第四行, 相对 X_ROW_* 再右移 X_MAN_ROW_SHIFT 像素 */
#define X_MAN_ROW_SHIFT 18                               /* 手动第四行额外右移 18 像素 */
#define X_MAN_ROW_LABEL (X_ROW_LABEL + X_MAN_ROW_SHIFT) /* 手动第四行 标签 */
#define X_MAN_ROW_VAL   (X_ROW_VAL   + X_MAN_ROW_SHIFT) /* 手动第四行 数值 */
/* 自动模式的第三/四行(Start / Stop)相对 X_ROW_* 再右移 X_AUTO_ROW_SHIFT 像素 */
#define X_AUTO_ROW_SHIFT 8                                   /* 自动第三/四行额外右移 8 像素 */
#define X_AUTO_ROW_LABEL (X_ROW_LABEL + X_AUTO_ROW_SHIFT)     /* 自动第三/四行 标签 */
#define X_AUTO_ROW_VAL   (X_ROW_VAL   + X_AUTO_ROW_SHIFT)     /* 自动第三/四行 数值 */
/* 清行宽度: 需同时覆盖 手动(到 X_MAN_ROW_VAL+23) 与 自动(到 X_AUTO_ROW_VAL+35) */
#define X_ROW_CLR_CHARS  15


/* 第三/四行高亮条(正在编辑的行, 白底黑字): 取代 '>' 与 '*' */
#define X_ROW_HL_X0     (X_SHIFT)
#define X_ROW_HL_W      (128 - X_ROW_HL_X0)

/* 第二行"当前模式"反相芯片: 普通黑底上的白底黑字(反白), 标题左右各留 2 像素 */
#define X_CHIP_MAN      (X_MODE_MAN - 2)                /* Manual 芯片(标题左右各留 2 像素) */
#define X_CHIP_W_MAN    (6 * 6 + 4)                     /* "Manual" 36 + 边距 4 */
#define X_CHIP_AUTO     (X_MODE_AUTO - 2)               /* Auto 芯片(标题左右各留 2 像素) */
#define X_CHIP_W_AUTO   (4 * 6 + 4)                     /* "Auto" 24 + 边距 4 */

/* ==================== 全局状态 ==================== */
static uint8_t  g_mode       = MODE_MANUAL;   /* 运行模式: 第二行反相芯片所标 */
static uint8_t  g_state      = ST_NORMAL;     /* 反相高亮所在行 */
static uint8_t  g_manualDuty = 0;             /* 手动模式转速 0~100; 代码默认 0%(刷完固件
                                                 首次上电即此值, 按 + 从启动阈值
                                                 FAN_START_DUTY 20% 起步); 之后由
                                                 Cfg_Load() 恢复掉电前保存的值 */
static int16_t  g_startTemp10 = 320;          /* 自动: 启动温度 x10℃ (32.0℃) */
static int16_t  g_stopTemp10  = 280;          /* 自动: 停止温度 x10℃ (28.0℃, 与启动相差 2.0℃) */
static uint8_t  g_fanOn      = 0;             /* 自动模式回差状态: 1=风扇已启动 */
static uint8_t  g_fanDuty    = 0;             /* 实际输出占空比 */
static uint8_t  g_rpmValid   = 0;             /* 1=测到转速信号; 0=没有(第一行显示 "-----") */
static uint32_t g_rpm        = 0;             /* 实测转速 (RPM), 5 位显示需要 >65535 的余量 */
static uint16_t g_rpmLastMs  = 0;             /* 上次测速的时刻(TIM2 的 1ms 计数) */
static int16_t  g_temp10     = NTC_TEMP_ERR;  /* 实时温度 x10℃ */
static int32_t  g_adcFilter  = 512;           /* ADC 滤波值 */
static uint8_t  g_screenOff  = 0;             /* 1=屏幕已关闭(只停显示) */
static uint8_t  g_screenAuto = 1;             /* 1=自动熄屏(默认), 0=常亮; 双击 OK 切换 */
static uint8_t  g_okPend     = 0;             /* OK 单击已挂起(等待可能的第二次单击) */
static uint16_t g_okPendMs   = 0;             /* 挂起时刻(Fan_Tick_Ms 计时) */
/* FG 脉冲计数: 由 EXTI7_0_IRQHandler(User/ch32v00x_it.c) 累加, 主循环每秒取走并清零 */
volatile uint16_t g_fanFgPulses = 0;
/* 绘制哨兵: 记录上次已写到屏上的值(变化时才刷屏; 置回哨兵即可强制重画) */
static int16_t  g_drawnTemp  = 0x7FFE;
static uint32_t g_drawnRpm   = 0xFFFFFFFF;
static uint8_t  g_drawnRpmValid = 0xFF;

/* ==================== 掉电保存 (Flash 最后 64 字节页) ====================
 *  保存 运行模式 + 手动转速 + 自动启停温度, 停电来电后恢复上次状态继续控温;
 *  刷入固件会整片擦除 Flash(含本页) -> 刷完首次上电为代码默认值: 手动模式 + 0%
 *  (风扇停转, 按 + 从启动阈值 FAN_START_DUTY 20% 起步)。
 *  存储位置: 0x00003FC0 (16K Flash 的最后一页, 链接脚本已留出 64B 不给代码)
 *  写入时机: 参数改动后 3 秒内无新改动才写(防抖, 避免连发调速反复擦写磨损 Flash);
 *            自动设置里长按"保存"则立即写。
 *  开机读取: magic 与校验和都对、且数值范围合法才采用, 否则保持代码默认值。
 *  (读走代码区别名 0x00000000, 擦写走 FPEC 要求的 0x08000000 别名) */
#define CFG_MAGIC        0x43464731u          /* 'CFG1' */
#define CFG_ADDR_XIP     0x00003FC0u          /* 读: 代码区别名   */
#define CFG_ADDR_APB     0x08003FC0u          /* 写: FPEC 要求别名 */
#define CFG_SIZE         64u                  /* 擦写单位(1 页)   */
#define CFG_SAVE_SEC     3                    /* 改动后 3s 无新改动才落盘 */

typedef struct {
    uint32_t magic;                           /* CFG_MAGIC          */
    uint8_t  mode;                            /* 手动/自动          */
    uint8_t  manualDuty;                      /* 手动转速 0~100     */
    int16_t  startTemp10;                     /* 启动温度 x10       */
    int16_t  stopTemp10;                      /* 停止温度 x10       */
    uint16_t cksum;                           /* 前 10 字节累加和   */
} Cfg_t;                                      /* 共 12 字节         */

static uint8_t g_cfgDirty = 0;                /* 1 = 参数改过待写 Flash */
static uint8_t g_cfgDelay = 0;                /* 距上次改动的秒数      */

static uint16_t Cfg_Cksum(const Cfg_t *c) {
    const uint8_t *p = (const uint8_t *)c;
    uint16_t s = 0;
    uint8_t  i;
    for (i = 0; i < 10; i++) s = (uint16_t)(s + p[i]);
    return s;
}

/* 开机恢复配置(参数非法/首次烧录时保持默认值: 手动 + 0%) */
static void Cfg_Load(void) {
    const Cfg_t *c = (const Cfg_t *)CFG_ADDR_XIP;

    if (c->magic != CFG_MAGIC) return;
    if (c->cksum != Cfg_Cksum(c)) return;
    if (c->mode > MODE_AUTO) return;
    if (c->manualDuty > 100) return;
    if (c->startTemp10 < TEMP_START_MIN || c->startTemp10 > TEMP_START_MAX) return;
    if (c->stopTemp10  < TEMP_STOP_MIN  || c->stopTemp10  > TEMP_STOP_MAX)  return;
    if ((int16_t)(c->startTemp10 - c->stopTemp10) < TEMP_GAP) return;

    g_mode       = c->mode;
    /* 旧配置可能存有低于启动阈值的转速(电机转不起来), 统一抬到 FAN_START_DUTY */
    g_manualDuty = (c->manualDuty != 0 && c->manualDuty < FAN_START_DUTY)
                   ? FAN_START_DUTY : c->manualDuty;
    g_startTemp10 = c->startTemp10;
    g_stopTemp10  = c->stopTemp10;
    printf("CFG loaded: mode=%s duty=%d start=%d stop=%d (0.1C)\r\n",
           (g_mode == MODE_AUTO) ? "AUTO" : "MANUAL",
           g_manualDuty, g_startTemp10, g_stopTemp10);
}

/* 立即写 Flash(先擦最后一页再写) */
static void Cfg_Save(void) {
    uint32_t buf[CFG_SIZE / 4];
    Cfg_t   *c = (Cfg_t *)buf;
    uint8_t *p = (uint8_t *)buf;
    uint8_t  i;

    c->magic      = CFG_MAGIC;
    c->mode       = g_mode;
    c->manualDuty = g_manualDuty;
    c->startTemp10 = g_startTemp10;
    c->stopTemp10  = g_stopTemp10;
    c->cksum      = Cfg_Cksum(c);
    for (i = sizeof(Cfg_t); i < CFG_SIZE; i++) p[i] = 0xFF;   /* 余量填 0xFF */

    __disable_irq();                          /* 擦写期间禁止中断取指 */
    FLASH_ROM_ERASE(CFG_ADDR_APB, CFG_SIZE);
    FLASH_ROM_WRITE(CFG_ADDR_APB, buf, CFG_SIZE);
    __enable_irq();

    g_cfgDirty = 0;
    g_cfgDelay = 0;
    printf("CFG saved: mode=%s duty=%d start=%d stop=%d (0.1C)\r\n",
           (g_mode == MODE_AUTO) ? "AUTO" : "MANUAL",
           g_manualDuty, g_startTemp10, g_stopTemp10);
}

/* 参数改动后调用: 3 秒内无新改动才真正写 Flash */
static void Cfg_MarkDirty(void) {
    g_cfgDirty = 1;
    g_cfgDelay = 0;
}

/* ==================== 按键扫描 (消抖 + 短按/长按/连发) ====================
 *  返回: bit0~2 短按(松开时产生) , bit3~5 长按(按住0.6s触发一次) , bit6~8 连发
 *  说明: 一次按住若已触发"长按", 松手时不再产生"短按"事件
 *  需在主循环中周期性调用(周期约 1ms)
 * ======================================================================= */
static uint8_t  keyStable   = 0;            /* 已消抖的按下状态, 1=按下 */
static uint8_t  keyTick[KEY_NUM];           /* 消抖计数 */
static uint16_t keyHold[KEY_NUM];           /* 按住时长计数 */
static uint8_t  keyLongDone[KEY_NUM];       /* 本次按住是否已触发过"长按" */

static uint8_t Key_ReadRaw(void) {
    uint8_t r = 0;
    if (GPIO_ReadInputDataBit(KEY_PLUS_PORT,  KEY_PLUS_PIN)  == 0) r |= (uint8_t)(1 << KEY_PLUS);
    if (GPIO_ReadInputDataBit(KEY_MINUS_PORT, KEY_MINUS_PIN) == 0) r |= (uint8_t)(1 << KEY_MINUS);
    if (GPIO_ReadInputDataBit(KEY_OK_PORT,    KEY_OK_PIN)    == 0) r |= (uint8_t)(1 << KEY_OK);
    return r;
}

static uint16_t Key_Scan(void) {
    uint8_t i, raw = Key_ReadRaw();
    uint16_t out = 0;

    for (i = 0; i < KEY_NUM; i++) {
        uint8_t bit = (uint8_t)(1 << i);

        if ((raw & bit) != (keyStable & bit)) {             /* 电平变化: 消抖计时 */
            keyTick[i]++;
            if (keyTick[i] >= KEY_DEBOUNCE_TICKS) {
                keyTick[i] = 0;
                keyStable ^= bit;
                if (keyStable & bit) {                      /* 确认按下 */
                    keyHold[i]     = 0;
                    keyLongDone[i] = 0;
                } else if (keyLongDone[i] == 0) {           /* 确认松开: 短按 */
                    out |= (uint16_t)(1 << i);
                }
            }
        } else {                                            /* 电平稳定 */
            keyTick[i] = 0;
            if (keyStable & bit) {                          /* 持续按住 */
                keyHold[i]++;
                if (keyLongDone[i] == 0) {
                    if (keyHold[i] >= KEY_LONG_PRESS_TICKS) {   /* 长按(只触发一次) */
                        keyLongDone[i] = 1;
                        keyHold[i]     = 0;
                        out |= (uint16_t)(1 << (i + 3));
                    }
                } else if (keyHold[i] >= KEY_REPEAT_PERIOD) {   /* 连发 */
                    keyHold[i] = 0;
                    out |= (uint16_t)(1 << (i + 6));
                }
            }
        }
    }
    return out;
}

static void Key_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Pin = KEY_PLUS_PIN | KEY_MINUS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;       /* 上拉输入, 按下为低 */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(KEY_PLUS_PORT, &GPIO_InitStructure);      /* PC3 / PC4 */

    GPIO_InitStructure.GPIO_Pin = KEY_OK_PIN;
    GPIO_Init(KEY_OK_PORT, &GPIO_InitStructure);        /* PD3 */
}
/* ==================== ADC + NTC ==================== */
static uint16_t ADC_ReadChannel(uint8_t ch) {
    uint16_t guard = 20000;

    ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_241Cycles);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) { if (--guard == 0) break; }
    return ADC_GetConversionValue(ADC1);
}

static void NTC_ADC_Init(void) {
    ADC_InitTypeDef  ADC_InitStructure = {0};
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);                   /* ADCCLK 需 <= 14MHz */

    GPIO_InitStructure.GPIO_Pin = NTC_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;       /* 模拟输入, 关闭数字部分 */
    GPIO_Init(NTC_PORT, &GPIO_InitStructure);

    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; /* 单次转换, 软件启动 */
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, NTC_ADC_CH, 1, ADC_SampleTime_241Cycles);
    ADC_Calibration_Vol(ADC1, ADC_CALVOL_50PERCENT);
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
}

/* ADC 码值 -> NTC 阻值 (Ω) */
static uint32_t NTC_CodeToOhm(uint16_t code) {
#if (NTC_AT_VDD == 1)
    /* VDD - NTC - ADC - R - GND : Vadc = VDD*R/(Rntc+R) */
    return (uint32_t)NTC_R_SERIES * (uint32_t)(NTC_ADC_FULL - code) / code;
#else
    /* VDD - R - ADC - NTC - GND : Vadc = VDD*Rntc/(Rntc+R) */
    return (uint32_t)NTC_R_SERIES * code / (uint32_t)(NTC_ADC_FULL - code);
#endif
}

/* NTC 阻值 -> 温度 x10℃ (查表 + 线性插值; 超出表格范围取端点值) */
static int16_t NTC_OhmToTemp10(uint32_t ohm) {
    uint8_t i;
    int32_t rHi, rLo, tHi, tLo;

    if (ohm >= (uint32_t)ntcRes10[0] * 10)                return ntcTemp10[0];
    if (ohm <= (uint32_t)ntcRes10[NTC_TAB_SIZE - 1] * 10) return ntcTemp10[NTC_TAB_SIZE - 1];

    for (i = 1; i < NTC_TAB_SIZE; i++) {                /* ntcRes10[] 随温度升高而减小 */
        if (ohm >= (uint32_t)ntcRes10[i] * 10) break;
    }

    rHi = (int32_t)ntcRes10[i - 1] * 10;                /* i-1: 较低温度, 阻值较大 */
    rLo = (int32_t)ntcRes10[i]     * 10;                /* i  : 较高温度, 阻值较小 */
    tLo = ntcTemp10[i - 1];
    tHi = ntcTemp10[i];

    return (int16_t)(tLo + (rHi - (int32_t)ohm) * (tHi - tLo) / (rHi - rLo));
}

/* 采样(含一阶滤波) + 温度换算 */
/* ==================== NTC 稳健采样（抗 PWM 开关干扰）====================
 * 连续采样 NTC_SAMPLE_COUNT 次，排序后去掉最大/最小各1个，取中间值平均。
 * 可有效抵抗风扇PWM开关瞬态耦合到ADC引脚的尖峰噪声。 */
#define NTC_SAMPLE_COUNT   7

static uint16_t NTC_ReadStable(void) {
    uint16_t buf[NTC_SAMPLE_COUNT];
    uint8_t i, j;
    uint16_t t;
    uint32_t sum = 0;

    for (i = 0; i < NTC_SAMPLE_COUNT; i++) {
        buf[i] = ADC_ReadChannel(NTC_ADC_CH);
    }

    /* 简单插入排序（样本数很小，开销可忽略） */
    for (i = 1; i < NTC_SAMPLE_COUNT; i++) {
        t = buf[i];
        j = i;
        while (j > 0 && buf[j - 1] > t) {
            buf[j] = buf[j - 1];
            j--;
        }
        buf[j] = t;
    }

    /* 去掉最大和最小，对中间 NTC_SAMPLE_COUNT-2 个求平均 */
    for (i = 1; i < NTC_SAMPLE_COUNT - 1; i++) {
        sum += buf[i];
    }
    return (uint16_t)(sum / (NTC_SAMPLE_COUNT - 2));
}

static void NTC_Update(void) {
    uint16_t code = NTC_ReadStable();

    g_adcFilter += ((int32_t)code - g_adcFilter) / NTC_FILTER_DIV;
    if (g_adcFilter < 4)                  g_adcFilter = 4;                  /* 限幅, 防除零 */
    if (g_adcFilter > (NTC_ADC_FULL - 4)) g_adcFilter = NTC_ADC_FULL - 4;

    if (code <= 2 || code >= (NTC_ADC_FULL - 1)) {
        g_temp10 = NTC_TEMP_ERR;                        /* 采到电源轨: NTC 开路/短路 */
    } else {
        g_temp10 = NTC_OhmToTemp10(NTC_CodeToOhm((uint16_t)g_adcFilter));
    }
}
/* ==================== 显示 ==================== */
/* 固定 6 字符宽温度字段: " 25.6C" / " -5.2C" / "-20.0C" / "--.-C" */
static void ShowTemp10(uint8_t x, uint8_t y, int16_t t10) {
    char num[7], buf[7];
    uint16_t a;
    uint8_t i, n = 0;

    if (t10 == NTC_TEMP_ERR) { OLED_ShowString(x, y, "--.-C"); return; }

    if (t10 < 0) { num[n++] = '-'; a = (uint16_t)(-t10); }
    else         { a = (uint16_t)t10; }

    if (a > 999) a = 999;                       /* 显示上限 99.9℃ */
    if (a >= 100) num[n++] = (char)('0' + (a / 100) % 10);
    num[n++] = (char)('0' + (a / 10) % 10);
    num[n++] = '.';
    num[n++] = (char)('0' + a % 10);

    for (i = 0; i < 6; i++) buf[i] = ' ';       /* 右对齐补齐到 6 字符 */
    for (i = 0; i < n; i++) buf[5 - n + i] = num[i];
    buf[5] = 'C';
    buf[6] = '\0';
    OLED_ShowString(x, y, buf);
}

/* 第二行: 两个模式标题(普通显示, 不铺高亮条) -- 开机画一次
 * 选中效果完全由 Draw_ModeChip() 的反相芯片给出 */
static void Draw_ModeRow(void) {
    OLED_ShowString(X_MODE_MAN,  Y_MODE, "Manual");
    OLED_ShowString(X_MODE_AUTO, Y_MODE, "Auto");
}

/* 静态界面 (开机画一次) */
static void Draw_Static(void) {
    OLED_ShowString(X_TEMP_LBL,   Y_TEMP, "T:");
    OLED_ShowString(X_RPM_LBL_Y0, Y_TEMP, "RPM:");
    Draw_ModeRow();
}

/* y0 左边: 实时温度 (数值变化时才写屏; 屏幕关闭时不写) */
static void Draw_Temp(void) {
    if (g_screenOff) return;
    if (g_temp10 == g_drawnTemp) return;
    g_drawnTemp = g_temp10;
    ShowTemp10(X_TEMP, Y_TEMP, g_temp10);
}

/* y0 右边: 风扇实测转速(FG 测速, 5 位右对齐); 没测到信号时显示 "-----" */
static void Draw_RPM(void) {
    if (g_screenOff) return;
    if (g_rpm == g_drawnRpm && g_rpmValid == g_drawnRpmValid) return;
    g_drawnRpm      = g_rpm;
    g_drawnRpmValid = g_rpmValid;
    if (!g_rpmValid) OLED_ShowString(X_RPM_VAL_Y0, Y_TEMP, "-----");
    else             OLED_ShowNum(X_RPM_VAL_Y0, Y_TEMP, g_rpm, 5);
}

/* 清一行(两种模式内容的公共区域, 留足余量防残影) */
static void Row_Clear(uint8_t y) {
    OLED_ShowString(X_ROW_LABEL, y, "               ");   /* X_ROW_CLR_CHARS = 15 字符 */
}

/* 第三行内容: 自动=启动温度(右移 8); 手动模式此行留空(转速改画到第四行) */
static void Draw_Row1(void) {
    if (g_mode == MODE_AUTO) {
        OLED_ShowString(X_AUTO_ROW_LABEL, Y_ROW1, "Start: ");
        ShowTemp10(X_AUTO_ROW_VAL, Y_ROW1, g_startTemp10);
    } else {
        Row_Clear(Y_ROW1);                            /* 手动: 整行清空 */
    }
}

/* 第四行内容: 手动=转速(右移 18), 自动=停止温度(右移 8) */
static void Draw_Row2(void) {
    Row_Clear(Y_ROW2);                                /* 先清, 覆盖两种模式 */
    if (g_mode == MODE_AUTO) {
        OLED_ShowString(X_AUTO_ROW_LABEL, Y_ROW2, "Stop : ");
        ShowTemp10(X_AUTO_ROW_VAL, Y_ROW2, g_stopTemp10);
    } else {
        OLED_ShowString(X_MAN_ROW_LABEL, Y_ROW2, "Speed: ");
        OLED_ShowNum(X_MAN_ROW_VAL, Y_ROW2, g_manualDuty, 3);
        OLED_ShowChar(X_MAN_ROW_VAL + 18, Y_ROW2, '%');
    }
}

/* 第三/四行内容随模式变化时重画 */
static uint8_t g_drawnMode = 0xFF;
static void Draw_Rows(void) {
    if (g_mode == g_drawnMode) return;
    Draw_Row1();
    Draw_Row2();
    g_drawnMode = g_mode;
}

/* 只刷数值 (改参数时用, 省 I2C 时间):
 *   手动=第四行转速; 自动=第三行启动温度 */
static void Draw_Row1Value(void) {
    if (g_mode == MODE_MANUAL) {
        OLED_ShowNum(X_MAN_ROW_VAL, Y_ROW2, g_manualDuty, 3);
        OLED_ShowChar(X_MAN_ROW_VAL + 18, Y_ROW2, '%');
    } else {
        ShowTemp10(X_AUTO_ROW_VAL, Y_ROW1, g_startTemp10);
    }
}

static void Draw_Row2Value(void) {
    ShowTemp10(X_AUTO_ROW_VAL, Y_ROW2, g_stopTemp10);
}

/* ---------------- 反相选中显示 (取代原 '>' 光标与 '*' 标记) ----------------
 *  原来的 '>' 与 '*' 总是同时出现在同一项上(状态切换时两者一起移动),
 *  所以"一个反相高亮"就能完整表达光标位置与当前运行模式, 不丢信息:
 *    第二行  : 普通黑底白字, 当前模式(=光标所在项)标题用反白芯片
 *              (白底黑字)突出, 标题左右各留 2 像素边距;
 *    第三/四行: 正在编辑的那一行整行反白(白底黑字), 其余行普通显示。
 * -------------------------------------------------------------------------- */
static uint8_t g_chipPos = SYM_NONE;        /* 第二行芯片: 0=Manual 1=Auto */

static const uint8_t chipX[2] = {X_CHIP_MAN,  X_CHIP_AUTO};
static const uint8_t chipW[2] = {X_CHIP_W_MAN, X_CHIP_W_AUTO};

/* 第二行: pos = MODE_MANUAL / MODE_AUTO (常显当前运行模式, 反白高亮) */
static void Draw_ModeChip(uint8_t pos) {
    char *txt;

    if (pos == g_chipPos) return;

    if (g_chipPos != SYM_NONE) {                             /* 撤销旧芯片: 清成黑底 + 普通标题 */
        txt = (g_chipPos == MODE_MANUAL) ? "Manual" : "Auto";
        OLED_WrFill(chipX[g_chipPos], Y_MODE, chipW[g_chipPos], 0x00);
        OLED_ShowString(chipX[g_chipPos] + 2, Y_MODE, txt);
    }
    if (pos != SYM_NONE) {                                   /* 新芯片: 白底黑字(反白) */
        txt = (pos == MODE_MANUAL) ? "Manual" : "Auto";
        OLED_WrFill(chipX[pos], Y_MODE, chipW[pos], 0xFF);
        OLED_ShowStringInv(chipX[pos] + 2, Y_MODE, txt);     /* 反相 -> 白底黑字 */
    }
    g_chipPos = pos;
}

/* 第三/四行: row = Y_ROW1 / Y_ROW2 / SYM_NONE (被编辑的行整行反相) */
static void Draw_RowHighlight(uint8_t row) {
    uint8_t old = g_rowHl;

    if (row == old) return;
    if (old != SYM_NONE) {                                   /* 1) 撤销旧高亮: 清底 + 普通重画 */
        OLED_WrFill(X_ROW_HL_X0, old, X_ROW_HL_W, 0x00);
        g_rowHl = SYM_NONE;                                  /* 复位后再重画才是普通字形 */
        if (old == Y_ROW1) Draw_Row1(); else Draw_Row2();
    }
    if (row != SYM_NONE) {                                   /* 2) 建立新高亮: 铺白底 + 反相重画 */
        OLED_WrFill(X_ROW_HL_X0, row, X_ROW_HL_W, 0xFF);
        g_rowHl = row;                                        /* 此后该行绘制自动取反 */
        if (row == Y_ROW1) Draw_Row1(); else Draw_Row2();
    }
    g_rowHl = row;
}

/* ---------------- 屏幕右下角: 屏幕工作模式图标 ----------------
 * 位置: 第 3 行 x=116..127(手动 "Speed:100%" 止于 97, 自动 "Stop" 止于 99, 不冲突)。
 * Zz = 自动熄屏(会睡觉) / ON = 常亮; 双击 OK 切换后立即刷新。
 * 设置模式的整行白条会经过此区域: 挂在 Draw_Symbols() 末尾, 白条建立/撤销后都会
 * 重画本图标, 且 OLED_ShowChar 在高亮行自动反相 -> 白条上显示黑字, 永远可读。 */
#define X_SCR_MODE_ICON     116      /* 图标起始 x(2 字符: 116 + 122, 刚好到屏幕右缘) */
static void Draw_ScrModeIcon(void) {
    char *txt = g_screenAuto ? "Zz" : "ON";
    OLED_ShowChar(X_SCR_MODE_ICON,     Y_ROW2, txt[0]);
    OLED_ShowChar(X_SCR_MODE_ICON + 6, Y_ROW2, txt[1]);
}

/* 刷新反相选中状态: 第二行当前模式芯片 + 第三/四行编辑高亮 */
static void Draw_Symbols(void) {
    Draw_ModeChip(g_mode);                       /* 当前模式常显(进入设置时也不消失) */
    Draw_RowHighlight((g_state == ST_SET_START) ? Y_ROW1 :
                      (g_state == ST_SET_STOP)  ? Y_ROW2 : SYM_NONE);
    Draw_ScrModeIcon();                          /* 右下角屏幕模式图标(最后画, 防白条覆盖) */
}


/* 强制整屏重画(唤醒屏幕时用): 复位各绘制哨兵后全部重画 */
static void Draw_ForceAll(void) {
    g_drawnTemp = 0x7FFE;                     /* 复位哨兵 -> 温度/转速必刷 */
    g_drawnRpm  = 0xFFFFFFFF;
    g_drawnRpmValid = 0xFF;
    g_drawnMode = 0xFF;                       /* 第三/四行重画 */
    g_chipPos   = SYM_NONE;                   /* 芯片状态未知, 让 Draw_ModeChip 重建 */
    g_rowHl     = SYM_NONE;                   /* 行高亮状态未知, 让 Draw_Symbols 重建 */
    OLED_Clear();
    Draw_Static();                            /* 第一行标签 + 第二行标题 */
    Draw_ModeChip(g_mode);                    /* 当前模式反白芯片 */
    Draw_Rows();                              /* 第三/四行内容 */
    Draw_Symbols();                           /* 芯片 / 行高亮 */
    Draw_Temp();
    Draw_RPM();
}

/* 关闭/唤醒屏幕: 只影响显示, 自动控温与 PWM 输出照常运行 */
static void Screen_Sleep(void) {
    if (g_screenOff) return;
    g_screenOff = 1;
    OLED_DisplayOff();                        /* 0xAE: 关显示, 显存保留 */
}

static void Screen_Wake(void) {
    if (!g_screenOff) return;
    g_screenOff = 0;
    OLED_DisplayOn();                         /* 0xAF: 恢复显示 */
    Draw_ForceAll();                          /* 整屏重画, 保证内容完整正确 */
}

/* 双击 OK: 常亮 <-> 自动熄屏 开关
 * 只切换工作模式, 不动当前屏幕亮灭:
 *   自动 -> 常亮: 保持点亮, 之后永不限时熄屏;
 *   常亮 -> 自动: 屏幕照常亮着, 重新开始 60s 无操作计时, 到点才自动熄 */
static void Screen_ModeToggle(void) {
    g_screenAuto = !g_screenAuto;
    if (g_screenAuto) {
        printf("Screen: auto sleep (60s idle)\r\n");
    } else {
        printf("Screen: always ON\r\n");
    }
    Draw_ScrModeIcon();                         /* 立即刷右下角图标 */
}

/* ==================== 开机画面 ====================
 *  上电(OLED_Init 之后)先显示开机画面, 停留 BOOT_LOGO_MS 再进入主界面;
 *  此时风扇电源仍关断(Fan_PWR_Init 在 main() 最前面已拉低 PC5)。
 *  延时按 100ms 分片并喂狗: 1.5s 虽未超 IWDG 的 1.67s 超时, 但 OLED 初始化
 *  本身已耗时一段时间, 分片最稳妥(同长按 OK 切模式处的做法)。 */
#define BOOT_LOGO_MS        1500         /* 开机画面停留时间(ms) */
#define SPLASH_FRAMES     15      /* 15 x 100ms = 1.5s */
#define SPLASH_BAR_X0     12
#define SPLASH_BAR_X1     116
#define SPLASH_BAR_Y0     20
#define SPLASH_BAR_Y1     27

static void Splash_Show(void) {
    static const char txt[] = "Smart Fan";
    uint8_t txtX = (uint8_t)((128 - 9 * 12) / 2);   /* 10 */
    uint8_t barW = (uint8_t)(SPLASH_BAR_X1 - SPLASH_BAR_X0 - 1);
    uint16_t w, i;

    for (w = 0; w <= SPLASH_FRAMES; w++) {
        /* 每帧重绘: 清缓冲区 -> 画标题 -> 画进度条 -> flush */
        for (i = 0; i < sizeof(g_splashBuf); i++) g_splashBuf[i] = 0;

        /* 标题: 大字 "Smart Fan", 水平居中, 占第 0~1 页 */
        for (i = 0; txt[i]; i++) Splash_PutCharBig(txtX + (uint8_t)(i * 12), 0, txt[i]);

        /* 进度条边框 */
        Splash_Rect(SPLASH_BAR_X0, SPLASH_BAR_Y0, SPLASH_BAR_X1, SPLASH_BAR_Y1, 1);

        /* 进度条填充 (0% -> 100%) */
        {
            uint8_t fillW = (uint8_t)((uint16_t)barW * w / SPLASH_FRAMES);
            if (fillW) Splash_FillRect(SPLASH_BAR_X0 + 1, SPLASH_BAR_Y0 + 1,
                                       SPLASH_BAR_X0 + fillW, SPLASH_BAR_Y1 - 1, 1);
        }

        Splash_Flush();

        /* 分片延时并喂狗: 10 x 100ms = 1.5s */
        for (i = 0; i < 10; i++) {
            Delay_Ms(10);
            IWDG_ReloadCounter();
        }
    }

    /* 进入主界面前清屏 */
    OLED_Clear();
}

/* ==================== 风扇电源 (PC5 = 12V MOS) ====================
 *  接法: PC5 -> MOS 栅极驱动, 高电平导通(风扇 12V 得电)。
 *  解决的三个问题:
 *   1) 上电风扇误转 : 复位后 GPIO 默认浮空, 栅极悬空可能误导通 -> main() 第一条
 *                      指令就把 PC5 配成推挽输出并拉低, OLED 初始化期间风扇必断电;
 *                      (稳妥起见硬件上栅极对地加 100k 下拉, 复位窗口也安全)
 *   2) 上电屏幕不亮 : 风扇启动浪涌把电源拉垮导致 MCU 复位/OLED 初始化失败 ->
 *                      现在风扇上电排在 OLED_Init 之后, 且软启动抑制浪涌;
 *   3) 风扇启动复位/闪屏: 合闸时先 0 占空比, 延时 FAN_PWR_BOOT_MS(2s) 让 12V 供电
 *                      稳定下来, 再从启动阈值 20% 按 20% 一级阶梯升速, 电机电流缓慢
 *                      上升, 不再瞬间打跌电源(电源被打跌会让 MCU 复位重启)。 */
#define FAN_PWR_PORT        GPIOC
#define FAN_PWR_PIN         GPIO_Pin_5       /* PC5 = MOS 栅极, 高电平导通 */
/* PD4 = 风扇电源 LED, 与 PC5 绑定在同一组宏里, 保证灯与 MOS 永远同步
 * (LED 高电平点亮: LED-电阻-PC5/PD4->GND 接法; 若你的接法相反, 对调下面两组) */
#define FAN_PWR_ON()        do { GPIO_SetBits(FAN_PWR_PORT, FAN_PWR_PIN);   \
                                 GPIO_SetBits(GPIOD, GPIO_Pin_4); } while (0)
#define FAN_PWR_OFF()       do { GPIO_ResetBits(FAN_PWR_PORT, FAN_PWR_PIN); \
                                 GPIO_ResetBits(GPIOD, GPIO_Pin_4); } while (0)
#define FAN_PWR_BOOT_MS     2000             /* 合闸(PC5 拉高)后先不动 PWM, 延时 2s 让 12V
                                                供电稳定, 之后才输出启动 PWM;
                                                实测电机起动电流叠加在合闸瞬间会把电源
                                                打跌 -> MCU 复位重启 */
#define FAN_SOFT_STEP_PCT   20               /* 软启动每级占空比 % (>= FAN_MIN_DUTY) */
#define FAN_SOFT_STEP_MS    30               /* 每级停留毫秒数 */
#define FAN_MODE_SW_DLY_MS  500              /* 切模式将引发电源通断动作时, 先延时再开关 */

static uint8_t g_fanPwrOn = 0;               /* 1 = PC5 已导通(风扇 12V 在电) */

/* PD6 = 模式 LED: 手动亮 / 自动灭 (高电平点亮, 接法相反时对调) */
#define MODE_LED_ON()       GPIO_SetBits(GPIOD, GPIO_Pin_6)
#define MODE_LED_OFF()      GPIO_ResetBits(GPIOD, GPIO_Pin_6)

/* 模式灯跟随 g_mode: 手动=亮, 自动=灭 (开机 Cfg_Load 后与每次切模式时调用) */
static void Mode_LED_Sync(void) {
    if (g_mode == MODE_MANUAL) MODE_LED_ON();
    else                       MODE_LED_OFF();
}

/* 尽早关断风扇电源并初始化两颗 LED: main() 第一句调用,
 * OLED 初始化前风扇绝不通电, 两灯先处于确定状态(灭) */
static void Fan_PWR_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_ResetBits(FAN_PWR_PORT, FAN_PWR_PIN);      /* 先写 ODR=0, 切成输出后立即是低 */
    GPIO_ResetBits(GPIOD, GPIO_Pin_4 | GPIO_Pin_6); /* 两颗 LED 初始为灭 */
    GPIO_InitStructure.GPIO_Pin   = FAN_PWR_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(FAN_PWR_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4 | GPIO_Pin_6;   /* PD4 电源灯 + PD6 模式灯 */
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    g_fanPwrOn = 0;
}

/* ==================== 风扇 PWM ==================== */
static void Fan_PWM_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    TIM_OCInitTypeDef TIM_OCInitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};
    uint16_t psc;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_TIM1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;               /* PD2 = TIM1_CH1 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* 按实际主频算分频, 使 PWM 频率接近 FAN_PWM_FREQ */
    psc = (uint16_t)(SystemCoreClock / ((uint32_t)FAN_PWM_FREQ * (FAN_PWM_ARR + 1)));
    if (psc > 0) psc--;

    TIM_TimeBaseInitStructure.TIM_Period = FAN_PWM_ARR;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

/* ---------------- 转速测量: TIM2 毫秒时基 + PC2 的 FG 中断计数 ---------------- */
/* TIM2 当 1kHz 自由运行计数器(不产生中断): 只用来量测测速窗口的真实长度,
 * 这样窗口里因为刷屏多花的几十毫秒不会把转速算偏。65.5s 才会回绕一次。 */
static void Fan_Tick_Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};
    uint16_t psc;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    psc = (uint16_t)(SystemCoreClock / 1000U);          /* 1 个计数 = 1ms */
    if (psc > 0) psc--;

    TIM_TimeBaseInitStructure.TIM_Period = 0xFFFF;      /* 自由运行(16 位) */
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
    TIM_Cmd(TIM2, ENABLE);
}
static uint16_t Fan_Tick_Ms(void) { return (uint16_t)TIM2->CNT; }

/* PC2 = FG 输入(上拉, 下降沿中断计数) */
static void Fan_FG_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    EXTI_InitTypeDef EXTI_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitStructure.GPIO_Pin = FAN_FG_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;       /* 集电极开路: 与外部 10k 上拉并联 */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(FAN_FG_PORT, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource2);
    EXTI_InitStructure.EXTI_Line = EXTI_Line2;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;   /* FG 拉低时记一个脉冲 */
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_EnableIRQ(EXTI7_0_IRQn);                       /* 中断服务在 ch32v00x_it.c */

    Fan_Tick_Init();
    g_rpmLastMs = Fan_Tick_Ms();                        /* 计时起点 */
}

/* 主循环里周期调用: 攒够 FAN_RPM_WINDOW_MS 才用这段窗口的脉冲数换算转速 */
static void Fan_UpdateRPM(void) {
    uint16_t nowMs = Fan_Tick_Ms();
    uint16_t winMs = (uint16_t)(nowMs - g_rpmLastMs);   /* 无符号相减, 回绕也是对的 */
    uint16_t pulses;
    uint32_t rpm;

    if (winMs < FAN_RPM_WINDOW_MS) return;              /* 还没攒够窗口: (脉冲留着继续累) */
    g_rpmLastMs = nowMs;

    __disable_irq();                                    /* 取走脉冲数并清零, 防与中断打架 */
    pulses = g_fanFgPulses;
    g_fanFgPulses = 0;
    __enable_irq();

    if (pulses == 0) {                                  /* 整个窗口一个脉冲都没有 -> 无信号 */
        if (g_rpmValid) printf("FG: no signal\r\n");
        g_rpmValid = 0;
        g_rpm      = 0;
    } else {
        rpm = (uint32_t)pulses * 60000UL / ((uint32_t)winMs * FAN_FG_PPR);
#if (FAN_RPM_FILTER_DIV > 1)
        if (g_rpmValid) {                               /* 一阶滤波: 显示值不会 ±30 地跳 */
            rpm = ((uint32_t)g_rpm * (FAN_RPM_FILTER_DIV - 1) + rpm + FAN_RPM_FILTER_DIV / 2)
                  / FAN_RPM_FILTER_DIV;
        }
#endif
        if (rpm > FAN_RPM_MAX) rpm = FAN_RPM_MAX;
        if (!g_rpmValid) printf("FG: signal ok\r\n");
        g_rpm      = rpm;
        g_rpmValid = 1;
    }
    Draw_RPM();
}

/* 设置输出占空比(带最低占空比限制); 值变化时才写 TIM
 * 电源时序(配合 PC5 MOS) -- 先通电稳定, 后出 PWM:
 *   -> 0 : 先把 PWM 归零, 再断 12V(风扇停转彻底不耗电)
 *   0 -> n: PC5 先拉高合闸, PWM 保持 0 -> 延时 FAN_PWR_BOOT_MS(2s) 让 12V 供电稳定
 *           -> 初始给启动阈值 FAN_START_DUTY(20%, 多数 PWM 风扇低于此值不转)
 *           -> 按 20%/级 阶梯升到目标(软启动, 抑制电机启动浪涌,
 *           避免电源被打跌造成 OLED 闪屏/MCU 复位) */
static void Fan_SetDuty(uint8_t duty) {
    uint8_t  step;
    uint16_t boot;

    if (duty != 0 && duty < FAN_MIN_DUTY) duty = FAN_MIN_DUTY;   /* 低占空比易堵转 */
    if (duty == g_fanDuty && (duty == 0 || g_fanPwrOn)) return;

    if (duty == 0) {                                             /* 停转: 先停 PWM 再断电 */
        TIM_SetCompare1(TIM1, 0);
        g_fanDuty = 0;
        if (g_fanPwrOn) {
            FAN_PWR_OFF();
            g_fanPwrOn = 0;
            printf("Fan power OFF\r\n");
        }
        return;
    }

    if (!g_fanPwrOn) {                                           /* 断电中 -> 合闸 + 软启动 */
        TIM_SetCompare1(TIM1, 0);                                /* 合闸瞬间必须 0 占空比 */
        g_fanDuty = 0;
        FAN_PWR_ON();
        g_fanPwrOn = 1;
        printf("Fan power ON, wait %dms -> %d%%\r\n", FAN_PWR_BOOT_MS, duty);
        /* 合闸后 PWM 保持 0, 延时 2s 让 12V 供电稳定下来再起电机:
         * 分片延时并喂狗(整段 2s 会吃掉 IWDG 1.67s 超时的大部分余量，故每 100ms 喂一次) */
        for (boot = 0; boot < FAN_PWR_BOOT_MS; boot += 100) {
            Delay_Ms(100);
            IWDG_ReloadCounter();
        }

        /* 初始启动值: 先给启动阈值 FAN_START_DUTY(20%), 保证风扇真正转起来 */
        TIM_SetCompare1(TIM1, (uint16_t)((uint32_t)FAN_START_DUTY * FAN_PWM_ARR / 100));
        g_fanDuty = FAN_START_DUTY;
        Delay_Ms(FAN_SOFT_STEP_MS);
        IWDG_ReloadCounter();

        for (step = (uint8_t)(FAN_START_DUTY + FAN_SOFT_STEP_PCT); step <= duty;
             step = (uint8_t)(step + FAN_SOFT_STEP_PCT)) {       /* 再阶梯上升到目标 */
            TIM_SetCompare1(TIM1, (uint16_t)((uint32_t)step * FAN_PWM_ARR / 100));
            g_fanDuty = step;
            Delay_Ms(FAN_SOFT_STEP_MS);
            IWDG_ReloadCounter();                                /* 阶梯段总长 <300ms, 顺手喂狗 */
        }
    }

    g_fanDuty = duty;
    TIM_SetCompare1(TIM1, (uint16_t)((uint32_t)duty * FAN_PWM_ARR / 100));
}

/* 自动(智能)模式: 按温差比例调速
 *   风扇停止时: 温度 >  启动温度 -> 启动, 占空比 = 温差(℃) x 10%
 *   风扇运行时: 温度 <= 停止温度 -> 停止
 *   运行中    : 占空比 = 温差 x 10%, 上限 100%, 不低于 FAN_MIN_DUTY
 * 例: 启动=32.0℃ / 停止=30.0℃ / FAN_MIN_DUTY=20:
 *     温度 32.1 -> 公式 1%  (受下限限制, 实际输出 20%)
 *     温度 34.0 -> 20%    38.0 -> 60%    42.0 及以上 -> 100%
 *     降温时反向递减: 38.0 -> 60%, 34.0 -> 20%, 直到 30.0℃ 以下停止 */
static uint8_t Fan_AutoDuty(void) {
    int16_t d;

    if (g_temp10 == NTC_TEMP_ERR) {                 /* NTC 异常 */
        g_fanOn = 0;
        return FAN_ON_SENSOR_ERR ? AUTO_DUTY_MAX : 0;
    }

    if (g_temp10 >= TEMP_FORCE_FULL) {              /* 过温保护: >=75.0℃ 直接满速 */
        g_fanOn = 1;                                /* 置运行态, 降温后自动回到曲线 */
        return AUTO_DUTY_MAX;
    }

    if (g_fanOn == 0) {                             /* 尚未启动 */
        if (g_temp10 <= g_startTemp10) return 0;    /* 没到启动温度 -> 保持停止 */
        g_fanOn = 1;                                /* 高于启动温度 -> 启动 */
    } else {                                        /* 已经启动 */
        if (g_temp10 <= g_stopTemp10) {             /* 降到停止温度 -> 停机 */
            g_fanOn = 0;
            return 0;
        }
    }

    d = (int16_t)(g_temp10 - g_startTemp10);        /* 温差, 单位 0.1℃ */
    if (d < 0) d = 0;                               /* 降温到启动温度以下: 按最小值运行 */

    d = (int16_t)(d * AUTO_SLOPE_PER_DEG / 10);     /* 温差(℃) x 10% */
    if (d > AUTO_DUTY_MAX) d = AUTO_DUTY_MAX;
    if (d < FAN_MIN_DUTY)  d = FAN_MIN_DUTY;        /* 运行中不低于最低占空比:
                                                       降温时必须保持转动, 直到停止温度 */

    return (uint8_t)d;
}

/* 按当前模式与温度刷新输出 */
static void Fan_Update(void) {
    if (g_mode == MODE_MANUAL) {
        g_fanOn = 0;                                /* 手动模式复位自动回差状态 */
        Fan_SetDuty(g_manualDuty);
    } else {
        Fan_SetDuty(Fan_AutoDuty());
    }
}

/* 预测切换到 newMode 后风扇电源(PC5)是否需要通/断动作(纯只读, 不改任何状态)。
 * 切换时 g_fanOn 会被清零, 所以自动模式按"尚未启动"的口径预测, 与实际执行一致。 */
static uint8_t Fan_PwrWillToggle(uint8_t newMode) {
    uint8_t willOn;

    if (newMode == MODE_MANUAL) {
        willOn = (g_manualDuty != 0);               /* 手动 0% = 断电 */
    } else {
        if (g_temp10 == NTC_TEMP_ERR)      willOn = (FAN_ON_SENSOR_ERR != 0);
        else if (g_temp10 >= TEMP_FORCE_FULL) willOn = 1;
        else willOn = (g_temp10 > g_startTemp10);   /* 未到启动温度 -> 停转断电 */
    }
    return (uint8_t)((willOn != 0) != (g_fanPwrOn != 0));
}
/* ==================== 按键处理 ==================== */
/* 手动模式: 调整风扇转速 (步进 MANUAL_STEP)
 * PWM 风扇有启动阈值(约 20% 占空比以下不转), 所以:
 *   0% + -> 直接给启动阈值 FAN_START_DUTY(不经过转不起来的 10%);
 *   <=阈值 - -> 直接回到 0%(停转断电), 不停留在低于阈值的无效挡位 */
static void Manual_Adjust(int8_t dir) {
    int16_t v;

    if (dir > 0) {
        if (g_manualDuty == 0) v = FAN_START_DUTY;               /* 停转 -> 直接给启动阈值 */
        else                   v = (int16_t)(g_manualDuty + MANUAL_STEP);
    } else {
        if (g_manualDuty <= FAN_START_DUTY) v = 0;               /* 阈值及以下 -> 直接停转 */
        else                                v = (int16_t)(g_manualDuty - MANUAL_STEP);
    }

    if (v < 0)   v = 0;
    if (v > 100) v = 100;
    if ((uint8_t)v == g_manualDuty) return;

    g_manualDuty = (uint8_t)v;
    Draw_Row1Value();
    if (g_mode == MODE_MANUAL) Fan_Update();
    Cfg_MarkDirty();                          /* 转速改动 -> 掉电保存 */
}

/* 自动设置模式: 调整 启动温度(第三行) / 停止温度(第四行) */
static void AutoSet_Adjust(int8_t dir) {
    if (g_state == ST_SET_START) {
        int16_t lo = (int16_t)(g_stopTemp10 + TEMP_GAP);    /* 至少比停止温度高 1.0℃ */
        int16_t v  = (int16_t)(g_startTemp10 + dir * TEMP_STEP);

        if (lo < TEMP_START_MIN) lo = TEMP_START_MIN;
        if (v < lo)              v  = lo;
        if (v > TEMP_START_MAX)  v  = TEMP_START_MAX;
        if (v == g_startTemp10)  return;

        g_startTemp10 = v;
        Draw_Row1Value();
        Cfg_MarkDirty();                      /* 阈值改动 -> 掉电保存 */
    } else if (g_state == ST_SET_STOP) {
        int16_t hi = (int16_t)(g_startTemp10 - TEMP_GAP);   /* 必须低于启动温度 */
        int16_t v  = (int16_t)(g_stopTemp10 + dir * TEMP_STEP);

        if (hi > TEMP_STOP_MAX) hi = TEMP_STOP_MAX;
        if (v > hi)             v  = hi;
        if (v < TEMP_STOP_MIN)  v  = TEMP_STOP_MIN;
        if (v == g_stopTemp10)  return;

        g_stopTemp10 = v;
        Draw_Row2Value();
        Cfg_MarkDirty();                      /* 阈值改动 -> 掉电保存 */
    } else {
        return;
    }

    if (g_mode == MODE_AUTO) Fan_Update();      /* 阈值改变立即生效 */
}

/* 按键事件处理 */
static void Ui_HandleKey(uint16_t ev) {
    uint8_t click = (uint8_t)(ev & KEY_EV_CLICK);           /* 短按 */
    uint8_t lng   = (uint8_t)((ev >> 3) & 0x07);            /* 长按 */
    uint8_t rep   = (uint8_t)((ev >> 6) & 0x07);            /* 连发 */
    uint8_t pm    = (uint8_t)((1 << KEY_PLUS) | (1 << KEY_MINUS));

    /* ---- 屏幕关闭时: 任意按键(+/-/OK, 短按/长按/连发)都能点亮 ----
     * 这次按键被"吞掉"不执行其它功能, 避免熄屏时误调速/误切换模式 ----
     * 但熄屏时双击 OK 仍可用: 唤醒的那次 OK 单击挂起参与双击检测 ---- */
    if (g_screenOff) {
        Screen_Wake();
        if (click & (1 << KEY_OK)) {           /* 熄屏唤醒的 OK 单击: 挂起等第二击 */
            g_okPend   = 1;
            g_okPendMs = Fan_Tick_Ms();
        }
        return;
    }

    /* ---- OK 单击: 不立即处理, 挂起 400ms 等第二次单击(OK 单击本无其它功能) ----
     * 双击 -> 屏幕常亮/自动熄屏开关; 单击 -> 超时静默清标志即可 ---- */
    if (click & (1 << KEY_OK)) {
        if (g_okPend) {                        /* 400ms 内第二击 -> 双击成立 */
            g_okPend = 0;
            Screen_ModeToggle();
            return;
        }
        g_okPend   = 1;                        /* 挂起等第二击(超时由主循环清零) */
        g_okPendMs = Fan_Tick_Ms();
        return;
    }

    /* ---- 长按 OK: 切换 手动/自动 模式 ---- */
    if (lng & (1 << KEY_OK)) {
        uint8_t newMode = (g_mode == MODE_MANUAL) ? MODE_AUTO : MODE_MANUAL;

        /* 这次切换若会改变风扇电源通断(PC5/PD4 动作), 先延时再执行:
         * 让本次按键事件完全"安定"后再合闸/断电, 避免开关动作与按键同刻发生。
         * 分片延时并在每片之间喂狗: 整段 500ms 一次性 Delay_Ms 不喂狗时,
         * 叠加后面的 I2C 刷屏最长会逼近 IWDG 的 1.67s 超时。 */
        if (Fan_PwrWillToggle(newMode)) {
            uint16_t w;
            printf("Mode switch will toggle fan power, delay %dms\r\n", FAN_MODE_SW_DLY_MS);
            for (w = 0; w < FAN_MODE_SW_DLY_MS; w += 100) {
                Delay_Ms(100);
                IWDG_ReloadCounter();
            }
        }

        g_mode  = newMode;
        g_state = ST_NORMAL;
        g_fanOn = 0;
        Cfg_MarkDirty();                      /* 模式切换 -> 掉电保存 */
        Mode_LED_Sync();                      /* PD6 模式灯: 手动亮 / 自动灭 */
        Draw_Rows();                    /* 第三/四行内容随模式变化 */
        Draw_Symbols();                 /* 芯片与行高亮复位 */
        Fan_Update();
        printf("Mode -> %s\r\n", (g_mode == MODE_MANUAL) ? "MANUAL" : "AUTO");
        return;
    }

    if (g_state == ST_NORMAL) {
        if (g_mode == MODE_MANUAL) {
            /* 手动: 短按/长按/连发 都用来调速 */
            if (click & (1 << KEY_PLUS))  Manual_Adjust(+1);
            if (click & (1 << KEY_MINUS)) Manual_Adjust(-1);
            if (lng   & (1 << KEY_PLUS))  Manual_Adjust(+1);
            if (lng   & (1 << KEY_MINUS)) Manual_Adjust(-1);
            if (rep   & (1 << KEY_PLUS))  Manual_Adjust(+1);
            if (rep   & (1 << KEY_MINUS)) Manual_Adjust(-1);
        } else {
            /* 自动: 短按无反应, 长按 +/- 进入设置模式 */
            if (lng & pm) {
                g_state = ST_SET_START;
                Draw_Symbols();         /* 第三行整行反相高亮 */
                printf("Enter set: start=%d stop=%d (0.1C)\r\n", g_startTemp10, g_stopTemp10);
            }
        }
    } else {
        /* 设置模式: 短按改数值, 长按 +/- 换行或保存 */
        if (click & (1 << KEY_PLUS))  AutoSet_Adjust(+1);
        if (click & (1 << KEY_MINUS)) AutoSet_Adjust(-1);

        if (lng & pm) {
            if (g_state == ST_SET_START) {
                g_state = ST_SET_STOP;                      /* 反相高亮跳到第四行 */
                printf("Set stop temp\r\n");
            } else {
                g_state = ST_NORMAL;                        /* 保存并回到第二行 */
                printf("Save: start=%d stop=%d (0.1C)\r\n", g_startTemp10, g_stopTemp10);
                Cfg_Save();                                 /* 立即写 Flash, 掉电不丢 */
            }
            Draw_Symbols();
        }
    }
}

/* ==================== 主程序 ==================== */
int main(void) {
    uint16_t tick = 0;
    uint16_t ev;
    uint16_t secDiv  = 0;                         /* 4 个温度周期 ≈ 1 秒 */
    uint16_t idleSec = 0;                         /* 连续无按键操作的秒数 */

    Fan_PWR_Init();                               /* 最先关风扇电源: OLED 初始化期间
                                                   * 风扇绝不通电, 消除上电误转/浪涌 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    printf("\r\nCH32V003 Smart Fan (NTC + SSD1306 + PWM)\r\n");
    printf("SystemClk:%d\r\n", (int)SystemCoreClock);

    OLED_Init();                                            /* OLED 显示 */
    Cfg_Load();                                             /* 恢复掉电前的模式/转速/温度(无/非法则用默认值) */
    Mode_LED_Sync();                                        /* PD6 跟随恢复出来的模式 */
    NTC_ADC_Init();                                         /* NTC 采样 */
    Fan_PWM_Init();                                         /* 风扇 PWM */
    Fan_FG_Init();                                          /* 风扇测速 (PC2 = FG) */
    Key_Init();                                             /* 按键 */

    /* ---- 看门狗: IWDG 用 LSI(≈40kHz), 40k/64=625Hz, 1044 计数 = 1.67s
     * 卡死时自动复位; 注意 CH32V003 的 IWDG 无 RCC 时钟使能, 直接写寄存器 ---- */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_PRESCALE);
    IWDG_SetReload(IWDG_RELOAD);
    IWDG_ReloadCounter();
    IWDG_Enable();

    /* ---- 开机画面: 停留 1.5s 后再加载主界面 ---- */
    Splash_Show();

    /* ---- 开机界面 ---- */
    OLED_Clear();
    Draw_Static();
    g_adcFilter = (int32_t)ADC_ReadChannel(NTC_ADC_CH);      /* 滤波初值, 避免慢慢爬升 */
    NTC_Update();
    NTC_Update();
    Draw_Temp();
    Draw_Rows();
    Draw_Symbols();
    Fan_Update();
    Draw_RPM();

    /* ---- 主循环 ---- */
    while (1) {
        /* 1. 按键扫描 (约 1ms 一次); 任意按键都清零"无操作"计时 */
        ev = Key_Scan();
        if (ev) { idleSec = 0; Ui_HandleKey(ev); }

        /* OK 单击挂起超时(400ms 内没有第二击) -> 静默清标志;
         * 熄屏唤醒来的挂起同样超时清零, OK 单击本身没有其它动作要补执行 */
        if (g_okPend && (uint16_t)(Fan_Tick_Ms() - g_okPendMs) >= DOUBLE_CLICK_MS) {
            g_okPend = 0;
        }

        /* 2. 温度采样 + 调速 (约 0.3~0.6s 一次) */
        if (++tick >= TEMP_PERIOD_TICKS) {
            tick = 0;
            NTC_Update();
            if (!g_screenOff) Draw_Temp();
            Fan_Update();
            Fan_UpdateRPM();                            /* 攒够测速窗口(约 1s)就算一次转速 */

            /* 3. 约 1 秒一次: 统计无按键时间, 超时熄屏(仅关显示; 常亮模式不熄) */
            if (++secDiv >= 4) {
                secDiv = 0;
                if (g_screenAuto && !g_screenOff && ++idleSec >= SCREEN_OFF_SEC) Screen_Sleep();

                /* 4. 掉电保存: 参数改动后 CFG_SAVE_SEC 秒无新改动才写 Flash
                 *    (防抖: 连发调速期间一直推迟, 省擦写次数延长 Flash 寿命) */
                if (g_cfgDirty && ++g_cfgDelay >= CFG_SAVE_SEC) Cfg_Save();
            }
        }

        IWDG_ReloadCounter();                       /* 喂狗(1.67s 超时) */
        Delay_Ms(1);
    }
}

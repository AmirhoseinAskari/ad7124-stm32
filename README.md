# AD7124 STM32 HAL Driver

STM32 HAL-based driver for the AD7124 24-bit Sigma-Delta ADC using SPI communication

## 🔧 Features
- ✅ **Full STM32 HAL compatibility** — supports all STM32 MCU series (F0–F7, G0/G4, H7, L0–L5, U0/U5)
- ⚡ **Interrupt-driven SPI data acquisition** — uses external interrupt on DOUT/RDY pin for efficient sampling without polling
- 📥 **Comprehensive register access & calibration** — full read/write support including filters, gain, and offset calibration
- 🛡️ **MISRA-C-style design** — clean, safe, and portable for embedded and safety-critical applications
- 🔄 **Modular and portable** — works with STM32CubeIDE, Keil, IAR, or Makefile-based environments
- 🧩 **Easy integration** — lightweight driver with minimal dependencies

## ⚙️ Getting Started

### 1. Configure SPI in STM32CubeMX or manually
- Mode: **Full-Duplex Master**
- Data Size: **8-bit**
- First Bit: **MSB First**
- Clock Polarity (CPOL): **High**
- Clock Phase (CPHA): **2nd Edge**

### 2. Configure Chip Select (CS) Pin (Optional)
- Add a **GPIO Output** pin for CS (Chip Select)

### 3. Configure an External interrupt for DOUT\RDY pin on ic and enable it in NVIC
- GPIO Mode: **External Interrupt Mode with Falling Edge trigger datection**
- GPIO Pull-up/Pull-down: **Pull Up**

### 4. Add the Driver to Your Project
- **Include** `ad7124.h` in your application code
- **Add** `ad7124.c` and `ad7124_config.c` to your compiler
- **Set the STM32 MCU series macro** in `ad7124_config.h`
- **Add the library folder** to your compiler’s include paths

### 4. Declare Global Variables
- In your application code, `extern` the following global variables:
```
extern AD7124_ConfigTypeDef    AD7124_Handler;
extern uint32_t                AD7124_ChannelSamples[AD7124_ENABLED_CHANNELS];
extern AD7124_RegisterTypeDef  configA;
```
- define a volatile variable for rdy flag
```
volatile uint8_t ad7124_rdy_flag = 0U;
```
- extern the volatile variable in stm32yyxx_it.c file and set it to 1 in the external interrupt function
```
extern volatile uint8_t ad7124_rdy_flag;

void EXTI_IRQHandler(void)
{   
    ad7124_rdy_flag = 1U;
}
```

## 🧪 API Reference
Each function returns an `AD7124_StatusTypeDef` status code.

### `AD7124_ReadRegister(...)`  
Reads a register value from the AD7124 via SPI.

### `AD7124_WriteRegister(...)`  
Writes data to a specified register of the AD7124 ADC via SPI.

### `AD7124_Reset(...)`  
Performs a hardware reset of the AD7124 ADC via SPI.

### `AD7124_Config(...)`
Initializes and configures the AD7124 ADC registers.

### `AD7124_ReadSampleData(...)`
Reads ADC conversion data from the AD7124 and updates the channel samples array.

### `AD7124_ErrorCheck(...)`
Checks the AD7124 ADC error status by reading the error register.

### `AD7124_Calibration(...)`
Performs gain and offset calibration on the AD7124 ADC.

## 💡 Example
A complete working example is available in

## 📜 License
This project is released under the [MIT License](./LICENSE).

## 👤 Author
**Amirhossein Askari**  
📧 theamiraskarii@gmail.com  
🔗 [GitHub Profile](https://github.com/AmirhoseinAskari)

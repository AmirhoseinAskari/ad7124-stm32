# AD7124 STM32 HAL Driver

STM32 HAL driver for the AD7124 24-bit Sigma-Delta ADC using SPI communication.

## 🔧 Features
- **Fully STM32 HAL compatible** — supports all STM32 MCU series (F0–F7, G0/G4, H7, L0–L5, U0/U5)
- **Interrupt-driven** — uses an external interrupt on DOUT/RDY to read samples instead of polling mode
- **Full register access & calibration** — supports reading/writing all registers, including internal calibration
- **MISRA-C-inspired design** — clean, safe, and portable code for embedded and safety-critical applications
- **Modular and portable** — compatible with STM32CubeIDE, Keil, IAR, and Makefile-based workflows
- **Easy integration** — minimal dependencies and simple setup

## ⚙️ Getting Started

### 1. Configure SPI (via STM32CubeMX or manually)

Set the following parameters:

- **Mode:** Full-Duplex Master  
- **Data Size:** 8-bit  
- **First Bit:** MSB First  
- **Clock Polarity (CPOL):** High  
- **Clock Phase (CPHA):** 2nd Edge

### 2. Configure the Chip Select (CS) Pin (Optional)

- Add a **GPIO Output** pin for the CS signal.

### 3. Configure External Interrupt for the DOUT/RDY Pin

- **GPIO Mode:** External Interrupt on Falling Edge  
- **Pull Configuration:** Pull-Up  
- **Enable the EXTI line** in NVIC settings

### 4. Add the Driver to Your Project

- **Include** `ad7124.h` in your application code  
- **Add** `ad7124.c` and `ad7124_config.c` to your compiler sources  
- **Define the STM32 MCU series macro** in `ad7124_config.h`  
- **Add the library folder** to your compiler's include paths 

### 5. Declare Global Variables

In your application code, `extern` the following global variables:

```
extern AD7124_ConfigTypeDef    AD7124_Handler;
extern uint32_t                AD7124_ChannelSamples[AD7124_ENABLED_CHANNELS];
extern AD7124_RegisterTypeDef  configA;
```
Also, define a flag for DOUT/RDY readiness:
```
volatile uint8_t ad7124_rdy_flag = 0U;
```
Then, in your stm32yyxx_it.c interrupt handler, do the following:
```
extern volatile uint8_t ad7124_rdy_flag;

void EXTI_IRQHandler(void)
{   
    ad7124_rdy_flag = 1U;
}
```

## 🧪 API Reference

Each function returns an AD7124_StatusTypeDef status code.

### `AD7124_ReadRegister(...)`  
Reads a register from the AD7124 via SPI

### `AD7124_WriteRegister(...)`  
Writes a value to a register on the AD7124

### `AD7124_Reset(...)`  
Performs a software reset of the AD7124 over SPI

### `AD7124_Config(...)`
Initializes and applies a full configuration to the AD7124

### `AD7124_ReadSampleData(...)`
Reads conversion results and updates the AD7124_ChannelSamples array

### `AD7124_ErrorCheck(...)`
Checks and returns AD7124 error status

### `AD7124_Calibration(...)`
Executes internal gain and offset calibration routines

## 💡 Example

A complete working example is available in the [example](./example) folder.

- [`example/main.c`](./example/main.c)  
- [`example/stm32yyxx_it.c`](./example/stm32yyxx_it.c)

## 📜 License
This project is licensed under the [MIT License](./LICENSE).

## 👤 Author
**Amirhossein Askari**  
📧 theamiraskarii@gmail.com  
🔗 [GitHub Profile](https://github.com/AmirhoseinAskari)

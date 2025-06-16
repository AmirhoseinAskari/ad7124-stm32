# AD7124 STM32 HAL Driver

STM32 HAL-based driver for the AD7124 24-bit Sigma-Delta ADC using interrupt-driven SPI communication

## 🔧 Features
- ✅ **Full STM32 HAL compatibility** — supports all STM32 MCU series (F0–F7, G0/G4, H7, L0–L5, U0/U5)
- 🛡️ **MISRA-C-style design** — clean, safe, and portable for embedded and safety-critical applications
- 🔄 **Modular and portable** — works with STM32CubeIDE, Keil, IAR, or Makefile-based environments
- 📥 Supports read/write access to all AD7124 registers

## ⚙️ Getting Started

### 1. Configure SPI in STM32CubeMX
- Mode: **Full-Duplex Master**
- Data Size: **8-bit**
- First Bit: **MSB First**
- Clock Polarity (CPOL): **High**
- Clock Phase (CPHA): **2nd Edge**

### 2. Configure Chip Select (CS) Pin (Optional)
- Add a **GPIO Output** pin for CS (Chip Select)

### 3. Add the Driver to Your Project
- **Include** `ad7124.h` in your application code
- **Add** `ad7124.c` and `ad7124_config.c` to your compiler
- **Set the STM32 MCU series macro** in `ad7124_config.h`
- **Add the library folder** to your compiler’s include paths

## 🧪 API Reference
Each function returns an `AD7124_StatusTypeDef` status code.

### `AD7124_ReadRegister(...)`  

### `AD7124_WriteRegister(...)`  

### `AD7124_Reset(...)`  

### `AD7124_Config(...)`

### `AD7124_ReadSampleData(...)`

### `AD7124_ErrorCheck(...)`

### `AD7124_Calibration(...)`

## 💡 Example
A complete working example is available in

## 📜 License
This project is released under the [MIT License](./LICENSE).

## 👤 Author
**Amirhossein Askari**  
📧 theamiraskarii@gmail.com  
🔗 [GitHub Profile](https://github.com/AmirhoseinAskari)

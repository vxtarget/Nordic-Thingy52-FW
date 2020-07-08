以下操作在Windows 10专业版系统下

# Nordic nRF52832环境搭建和编译步骤

**安装MDK**

MDK 是 Microcontroller Development Kit 的缩写，我们用这个开发环境来开发、调试、仿真Nordic芯片的程序。

软件版本：MDK-ARM Professional Version:5.23。

**安装Nordic设备支持包**

[官网地址：]https://developer.nordicsemi.com/nRF5_SDK/pieces/nRF_DeviceFamilyPack/

版本选择：NordicSemiconductor.nRF_DeviceFamilyPack8.29.0.pack

安装完MDK之后，直接双击安装Nordic支持包。也可在MDK 环境下单击“Pack Installer”按钮，在弹出的对话框内，选择"File - > Import"导入Nordic设备支持包。

**下载Nordic工程文件**

git clone https://github.com/haobtc/Nordic-Thingy52-FW.git

git checkout bixin

**安装JLINK驱动**

下载 Setup_JlinkARM_V614.exe文件，双击安装。

**编译Nordic设备bootloader和firmware**

**bootloader:**使用MDK打开根目录下的dfu文件夹下的secure_bootloader_ble_s132_pca10040.uvprojx文件。打开工程文件后按F7键或点击“Build”按钮执行编译过程。执行完毕后在\dfu\_build文件夹下生成nrf52832_xxaa_s132.hex文件为bootloader文件。

**firmware:**使用MDK打开根目录下的project文件夹下的ble_nfc.uvprojx文件，打开工程文件后按F7键（或点击“Build”按钮）执行编译过程。执行完毕后在\project\_build文件夹下生成nrf52832_xxaa.hex文件为firmware文件

**编译在线调试版本**

在MDK编译环境内按“Alt+F7"（或右击左侧工程根目录），弹出”Options for Target'52832_xxaa'“界面，在”C/C++“栏内的"Define"项目中，去掉“BUTTONLESS_ENABLED”宏定义。编译完成后按“Ctrl + F5”（或单击“Start/Stop Debug Session”按钮）启动调试功能。

**编译J-Link RTT Viewer调试版本**

在\Nordic-Thingy52-FW\app文件夹内，打开sdk_config.h文件(或在工程目录下双击sdk_config.h)，分别把“NRF_LOG_BACKEND_RTT_ENABLED”、“NRF_LOG_ENABLED”、“NRF_LOG_STR_FORMATTER_TIMESTAMP_FORMAT_ENABLED”设置为1，然后编译。把编译后的文件下载到Nordic设备中。然后打开J-LINK RTT Viewer工具，在对话框内选择“USB”、“nRF52832_xxAA”、“SWD”、“4000”、“Auto Detection”各个选项，点击OK，会出现打印调试信息界面。


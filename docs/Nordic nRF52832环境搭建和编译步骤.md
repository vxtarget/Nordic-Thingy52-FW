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

**使用Nordic官方APP测试**

github地址：https://github.com/NordicSemiconductor/Android-nRF-Connect.git

**（1）开始扫描**

点击右上角的SCAN文字就可以就可以扫描周边所有广播状态的蓝牙低能耗外设，并显示在SCANNER选项卡下的列表中。

**（2）停止扫描**

点击界面右上角SCAN文字以后，文字会变为**“STOP SCANNING”，**点击它就可以停止扫描。在停止以前扫描出来的设备，仍然会显示在列表中。

**（3）扫描结果：**在列表中我们可以看到扫描出来名字为BixinKeyxxxxxxxxxx的Bluetooth（蓝牙）低功耗设备。点击其中的区域，我们可以看到具体的设备信息

​	Device type:**LE only**

​	Advertising type:**Legacy**

​	Flags:**GeneralDiscoverable,BrEdrNotSupported**

​	Complete list of 16-bit Service UUIDs:**0x180A,0x180F,0x0001**

​	Complete Local Name:**BixinKeyxxxxxxxxxx**

**（4）过滤功能：**

​     **1、广播名称或MAC地址过滤。**

​      默认情况下是 “No filter”的。点击“No filter”最右边的箭头图标，可以打开过滤条件选项界面。可以根据bluetooth设备名字（BixinKey关键字）或者mac地址进行过滤。

​     **2、广播数据类型进行过滤**

​      点击编辑框后面的三个竖点，可以看到**六种数据类型**：DFU、nRF Beacon、iBeacon 、Eddystone、 Physical Web 、HRM。其中DFU又分为Legacy DFU和 Secure DFU两种类型。最后可以通过点击进行选择。

​     **3、根据rssi（信号强度）进行过滤**

​       取值范围是（-40dBm 到 -100 dBm之间)。

   **(5)发送和接收数据**

​		发送数据：点击UUID：00000002-0000-1000-8000-00805f9b34fb服务后面的向上箭头，在弹出的对话框中输入要发送的数据，点击右下角SEND字符即可发送。

​       接收数据：电机架UUID：00000003-0000-1000-8000-00805f9b34fb服务后面的向下的三个箭头按钮，可以打开Notify使能，这样一旦蓝牙设备给手机返回数据时就可以自动捕捉到。
# Nordic签名固件及生成升级固件

Secure DFU采用签名（Sign）方式保证DFU的安全。直观上，它解决了一个安全问题。

**pc-nrfutil工具**

pc-nrfutil是Nordic提供的命令行工具，它用于执行以下任务：

- 生成公钥、私钥
- 生成升级包
- 生成bootloader_settings.hex
- 执行BLE DFU或串口DFU

nrfutil是开源的python项目，托管在github上，如果系统中已安装了python，也可以通过pip命令进行安装：

```python
pip install nrfutil
```

也可直接下载release版本 https://github.com/NordicSemiconductor/pc-nrfutil.git 并放到`C:\nordic\nrfutil`中，并将该目录加入系统环境变量，方便在cmd中调用。

**生成公钥、私钥**

在当前目录下生成一个唯一的私钥（private_key.pem）：

```bash
nrfutil keys generate private_key.pem
```

利用私钥，可以生成一个公钥：

```bash
nrfutil keys display 
    --key pk 
    --format code private_key.pem 
    --out_file dfu_public_key.c
```

执行完毕，生成的公钥保存在dfu_public_key.c文件中。Bootloader工程中会使用公钥dfu_public_key.c，这样Bootloader就可以利用公钥验证签名。

**生成签名固件**

使用pc-nrfutil生成升级压缩包的命令为：

nrfutil pkg generate --hw-version 52 --sd-req 0xCB --application-version 0x03 --application nrf52832_xxaa_app.hex --key-file key1 nrf52832_xxaa_app.zip

**签名固件信息**

nrf52832_xxaa_app.zip内包含3个文件

- app.bin

- app.dat

- manifest.json

  manifest.json是清单文件，记录了dfu_pkg.zip中包含哪些文件。 app.bin是app.hex去掉地址信息后的二进制文件。 app.dat就是Init Packet的二进制形态。

Init Packet中包含以下内容：

- Image type & size & hash (APP, BTL, SD or combination)

- Version of FW/HW

- sd_req

- Signature type & bytes

  通过pc-nrfutil命令可以查看这些信息内容：

  ```
  nrfutil pkg display nrf52832_xxaa_app.zip
  ```

  ```
  DFU Package: <nrf52832_xxaa_app.zip>:
  |
  |- Image count: 1
  |
  |- Image #0:
     |- Type: application
     |- Image file: nrf52832_xxaa_app.bin
     |- Init packet file: nrf52832_xxaa_app.dat
        |
        |- op_code: INIT
        |- signature_type: ECDSA_P256_SHA256
        |- signature (little-endian): 423116ed4791af178d1bb9a4cde9f2af37a70e924f6b3930961e56083c0d7d2c06f0e8480f8ddde3b5b1feef5b0d085f69035d90b204c364b9f6a07dc4b52f78
        |
        |- fw_version: 0x00000003 (3)
        |- hw_version 0x00000034 (52)
        |- sd_req: 0xCB
        |- type: APPLICATION
        |- sd_size: 0
        |- bl_size: 0
        |- app_size: 82564
        |
        |- hash_type: SHA256
        |- hash (little-endian): c31c700cbb16d7b7514860069bf3ee82ce1d16637775ce86454231d7272cd854
        |
        |- boot_validation_type: ['VALIDATE_GENERATED_CRC']
        |- boot_validation_signature (little-endian): ['']
        |
        |- is_debug: False
  ```
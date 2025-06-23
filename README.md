# SX1280 Linux Driver

This a work-in-progress Linux kernel driver for the Semtech SX1280 Transceiver.

## Protocol

## Device Tree

The following is an example device tree fragment.

```dts
&spi {
  status = "okay";

  lora_device@0 {
    compatible = "semtech,sx1280";
    reg = <0>;

    dio0 = <&gpio1 10 GPIO_ACTIVE_HIGH>;
    dio1 = <&gpio1 11 GPIO_ACTIVE_HIGH>;
    dio2 = <&gpio1 12 GPIO_ACTIVE_HIGH>;
    dio-interrupt = <1>;
  };
};
```

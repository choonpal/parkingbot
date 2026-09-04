# Third-party notices

The root [MIT License](LICENSE) is the license notice for this project's own
source. It does not replace the notices or licenses of bundled third-party
components or model weights.

| Component | Repository location | Existing license information |
|---|---|---|
| Ultralytics YOLO11n-Seg-derived vehicle checkpoint | `ros2/cooperative_parking_robot/models/parking_vehicle_yolo11n_seg.pt` | Checkpoint metadata: `AGPL-3.0`; Ultralytics version `8.4.123` |
| STMicroelectronics STM32F4 HAL driver | `stm32/parking_robot/Drivers/STM32F4xx_HAL_Driver/` | BSD 3-Clause notice in the component's `LICENSE.md` |
| Arm CMSIS | `stm32/parking_robot/Drivers/CMSIS/` | Apache License 2.0 in the component's `LICENSE.txt` |
| ST STM32F4 CMSIS device files | `stm32/parking_robot/Drivers/CMSIS/Device/ST/STM32F4xx/` | Apache License 2.0 in the component's `LICENSE.md` |

## Model weights

The packaged checkpoint is not relicensed as MIT. Its recorded origin, hash,
training setup and dataset limitations are in
[PACKAGED_YOLO11_SEG_MODEL.md](ros2/cooperative_parking_robot/docs/PACKAGED_YOLO11_SEG_MODEL.md).
The source training archive and dataset are external project assets; the
repository contains the selected checkpoint, not the complete training dataset.
No separate commercial license or dataset redistribution permission is asserted
by this notice. Check the applicable upstream and dataset terms for a planned use.

## Preserved component notices

- [STM32 HAL license](stm32/parking_robot/Drivers/STM32F4xx_HAL_Driver/LICENSE.md)
- [CMSIS license](stm32/parking_robot/Drivers/CMSIS/LICENSE.txt)
- [STM32F4 CMSIS device license](stm32/parking_robot/Drivers/CMSIS/Device/ST/STM32F4xx/LICENSE.md)

Runtime dependencies installed separately (including ROS 2, OpenCV, PyTorch and
Ultralytics) retain their own licenses. This file identifies bundled components;
it is not a complete dependency inventory for every deployed machine.

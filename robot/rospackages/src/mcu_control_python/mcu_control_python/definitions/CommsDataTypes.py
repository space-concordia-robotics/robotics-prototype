UINT8_SIZE = 1
UINT16_SIZE = 2
UINT32_SIZE = 4
FLOAT32_SIZE = 4
INT8_SIZE = 1
INT16_SIZE = 2

ARG_UINT8_ID = 0
ARG_UINT16_ID = 1
ARG_UINT32_ID = 2
ARG_FLOAT32_ID = 3
ARG_STRING_ID = 4
ARG_INT8_ID = 5
ARG_INT16_ID = 6

ARG_ARM_IDLE = 0
ARG_ARM_FORWARD = 1
ARG_ARM_BACKWARD = 2

ARG_UINT8 = (ARG_UINT8_ID, UINT8_SIZE)
ARG_UINT16 = (ARG_UINT16_ID, UINT16_SIZE)
ARG_UINT32 = (ARG_UINT32_ID, UINT32_SIZE)
ARG_FLOAT32 = (ARG_FLOAT32_ID, FLOAT32_SIZE)
ARG_STRING = (ARG_STRING_ID, None)  # todo handle strings
ARG_INT8 = (ARG_INT8_ID, INT8_SIZE)
ARG_INT16 = (ARG_INT16_ID, INT16_SIZE)

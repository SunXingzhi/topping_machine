#ifndef TEST_MAIN_H
#define TEST_MAIN_H

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    HAL_OK = 0,
    HAL_ERROR,
    HAL_BUSY,
    HAL_TIMEOUT
} HAL_StatusTypeDef;

typedef enum
{
    DISABLE = 0,
    ENABLE = 1
} FunctionalState;

typedef struct
{
    int dummy;
} CAN_TypeDef;

#define CAN1 ((CAN_TypeDef *)0x40000000u)

typedef enum
{
    HAL_CAN_STATE_RESET = 0,
    HAL_CAN_STATE_READY,
    HAL_CAN_STATE_LISTENING,
    HAL_CAN_STATE_SLEEP_PENDING,
    HAL_CAN_STATE_SLEEP_ACTIVE,
    HAL_CAN_STATE_ERROR
} HAL_CAN_StateTypeDef;

typedef struct
{
    CAN_TypeDef *Instance;
    HAL_CAN_StateTypeDef State;
} CAN_HandleTypeDef;

typedef struct
{
    uint32_t StdId;
    uint32_t ExtId;
    uint32_t IDE;
    uint32_t RTR;
    uint32_t DLC;
    FunctionalState TransmitGlobalTime;
} CAN_TxHeaderTypeDef;

typedef struct
{
    uint32_t StdId;
    uint32_t ExtId;
    uint32_t IDE;
    uint32_t RTR;
    uint32_t DLC;
    uint32_t Timestamp;
    uint32_t FilterMatchIndex;
} CAN_RxHeaderTypeDef;

typedef struct
{
    uint32_t FilterBank;
    uint32_t FilterMode;
    uint32_t FilterScale;
    uint32_t FilterIdHigh;
    uint32_t FilterIdLow;
    uint32_t FilterMaskIdHigh;
    uint32_t FilterMaskIdLow;
    uint32_t FilterFIFOAssignment;
    uint32_t FilterActivation;
    uint32_t SlaveStartFilterBank;
} CAN_FilterTypeDef;

#define CAN_ID_EXT              4U
#define CAN_RTR_DATA            0U
#define CAN_FILTERMODE_IDMASK   0U
#define CAN_FILTERSCALE_32BIT   1U
#define CAN_FILTER_FIFO0        0U
#define CAN_FILTER_ENABLE       1U
#define CAN_RX_FIFO0            0U
#define CAN_RX_FIFO1            1U

extern CAN_HandleTypeDef hcan;

HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *hcan, const CAN_FilterTypeDef *sFilterConfig);
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan, const CAN_TxHeaderTypeDef *pHeader, const uint8_t aData[], uint32_t *pTxMailbox);
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *hcan, uint32_t TxMailboxes);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan, uint32_t RxFifo, CAN_RxHeaderTypeDef *pHeader, uint8_t aData[]);
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(const CAN_HandleTypeDef *hcan);
uint32_t HAL_CAN_IsTxMessagePending(const CAN_HandleTypeDef *hcan, uint32_t TxMailboxes);
uint32_t HAL_CAN_GetRxFifoFillLevel(const CAN_HandleTypeDef *hcan, uint32_t RxFifo);
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t Delay);

#endif
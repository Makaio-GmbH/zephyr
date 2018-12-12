//
// Created by Christoph Schramm on 08.12.18.
//
#include <logging/log.h>
#include <user.h>
#include "cc3100.h"

LOG_MODULE_REGISTER(cc3100_test, 4);
#define UartWrite LOG_DBG

#define APPLICATION_VERSION "1.2.0"

#define TEST_SPI_BEGIN                  "Spi Test Begin"
#define TEST_SPI_OPEN_PASSED            "Spi Open Passed"
#define TEST_DEVICE_DISABLE_PASSED      "Device Disable Passed"
#define TEST_DEVICE_ENABLE_PASSED       "Device Enable Passed"
#define TEST_SPI_WRITE_PASSED           "Spi Write Passed"
#define TEST_SPI_READ_PASSED            "Spi Read Passed"
#define TEST_SPI_IRQ_PASSED             "Host IRQ Passed"
#define SPI_TEST_COMPLETED              "Spi Test Completed"
#define SPI_INIT_COMPLETE_READ_PASSED   "Spi Init read complete Passed"

#define H2N_SYNC_PATTERN     {0x4321,0x34,0x12}
#define H2N_CNYS_PATTERN     {0x8765,0x78,0x56}

#define H2N_DUMMY_PATTERN    0xFFFFFFFF
#define N2H_SYNC_PATTERN     0xABCDDCBA
#define SYNC_PATTERN_LEN     sizeof(_u32)

/* SPI bus issue
*  2 LSB of the N2H_SYNC_PATTERN are for sequence number
*  only in SPI interface
*  support backward sync pattern
*/
/* Bits 0..1    - use the 2 LBS for seq num */
#define N2H_SYNC_PATTERN_SEQ_NUM_BITS       ((_u32)0x00000003)
/* Bit  2       - sign that sequence number exists in the sync pattern */
#define N2H_SYNC_PATTERN_SEQ_NUM_EXISTS     ((_u32)0x00000004)
/* Bits 3..31   - constant SYNC PATTERN */
#define N2H_SYNC_PATTERN_MASK               ((_u32)0xFFFFFFF8)
/* Bits 7,15,31 - ignore the SPI (8,16,32 bites bus) error bits */
#define N2H_SYNC_SPI_BUGS_MASK   ((_u32)0x7FFF7F7F)
#define BUF_SYNC_SPIM(pBuf)      ((*(_u32 *)(pBuf)) & N2H_SYNC_SPI_BUGS_MASK)
#define N2H_SYNC_SPIM            (N2H_SYNC_PATTERN    & N2H_SYNC_SPI_BUGS_MASK)

#define N2H_SYNC_SPIM_WITH_SEQ(TxSeqNum)                                      \
    ((N2H_SYNC_SPIM & N2H_SYNC_PATTERN_MASK) | N2H_SYNC_PATTERN_SEQ_NUM_EXISTS \
    | ((TxSeqNum) & (N2H_SYNC_PATTERN_SEQ_NUM_BITS)))

#define MATCH_WOUT_SEQ_NUM(pBuf)   (BUF_SYNC_SPIM(pBuf) == N2H_SYNC_SPIM)

#define MATCH_WITH_SEQ_NUM(pBuf, TxSeqNum)                                     \
    (BUF_SYNC_SPIM(pBuf) == (N2H_SYNC_SPIM_WITH_SEQ(TxSeqNum)))

#define N2H_SYNC_PATTERN_MATCH(pBuf, TxSeqNum)                                \
    (                                                                         \
        ((*((_u32 *)pBuf) & N2H_SYNC_PATTERN_SEQ_NUM_EXISTS) &&             \
            (MATCH_WITH_SEQ_NUM(pBuf, TxSeqNum))) ||                          \
        (!(*((_u32 *)pBuf) & N2H_SYNC_PATTERN_SEQ_NUM_EXISTS) &&            \
            ( MATCH_WOUT_SEQ_NUM(pBuf)))                                      \
    )

#define SL_OPCODE_DEVICE_INITCOMPLETE                                   0x0008

#define SUCCESS         0

typedef unsigned long   _u32;

/* Status values - These are used to set/reset the corresponding bits in a 'status_variable' */
typedef enum
{
	TEST_STAGE_SPI_TEST_BEGIN,
	TEST_STAGE_SPI_OPEN_PASSED,
	TEST_STAGE_DISABLE_PASSED,
	TEST_STAGE_ENABLE_PASSED,
	TEST_STAGE_SPI_WRITE_PASSED,
	TEST_STAGE_SPI_READ_PASSED,
	TEST_STAGE_SPI_IRQ_PASSED,
	TEST_STAGE_INIT_COMPLETE_READ_PASSED,
	TEST_STAGE_SPI_TEST_COMPLETE
}testStage_e;

typedef struct
{
	unsigned short  Short;
	unsigned char   Byte1;
	unsigned char   Byte2;
}_SlSyncPattern_t;


/*
 * GLOBAL VARIABLES -- Start
 */
testStage_e g_testStage = TEST_STAGE_SPI_TEST_BEGIN;

int g_irqCounter = 0;

unsigned char TxSeqNum = 0;


/*
 * GLOBAL VARIABLES -- End
 */

/*
 * STATIC FUNCTION DEFINITIONS -- Start
 */

static void SetTestStage(testStage_e stage);
/*
 * STATIC FUNCTION DEFINITIONS -- End
 */

/*!
    \brief register an interrupt handler for the host IRQ

    \param[in]      none

    \return         upon successful registration, the function shall return 0.
                    Otherwise, -1 shall be returned

    \sa
    \note           If there is already registered interrupt handler, the
                    function should overwrite the old handler with the new one
    \warning
*/
void DiagIrqHandler()
{
	g_irqCounter++;
}

/*!
    \brief Update the state machine and write the output on Application UART

    \param[in]      stage - New state of the application

    \return         none

    \warning
*/
static void SetTestStage(testStage_e stage)
{
	g_testStage = stage;
	switch(stage)
	{
		case TEST_STAGE_SPI_TEST_BEGIN:
			LOG_DBG(TEST_SPI_BEGIN);
			LOG_DBG("\r\n");
			break;
		case TEST_STAGE_SPI_OPEN_PASSED:
			LOG_DBG(TEST_SPI_OPEN_PASSED);
			LOG_DBG("\r\n");
			break;
		case TEST_STAGE_DISABLE_PASSED:
			LOG_DBG(TEST_DEVICE_DISABLE_PASSED);
			LOG_DBG("\r\n");
			break;
		case TEST_STAGE_ENABLE_PASSED:
			LOG_DBG(TEST_DEVICE_ENABLE_PASSED);
			LOG_DBG("\r\n");
			break;
		case TEST_STAGE_SPI_WRITE_PASSED:
			LOG_DBG(TEST_SPI_WRITE_PASSED);
			LOG_DBG("\r\n");
			break;
		case TEST_STAGE_SPI_READ_PASSED:
			LOG_DBG(TEST_SPI_READ_PASSED);
			LOG_DBG("\r\n");
			break;
		case TEST_STAGE_SPI_IRQ_PASSED:
			LOG_DBG(TEST_SPI_IRQ_PASSED);
			LOG_DBG("\r\n");
			break;
		case TEST_STAGE_INIT_COMPLETE_READ_PASSED:
			LOG_DBG(SPI_INIT_COMPLETE_READ_PASSED);
			LOG_DBG("\r\n");
			break;
		case TEST_STAGE_SPI_TEST_COMPLETE:
			LOG_DBG(SPI_TEST_COMPLETED);
			LOG_DBG("\r\n\r\n");
			break;
	}
}

/*!
    \brief Test the spi communication

    \param[in]      none

    \return         upon successful, the function shall return 0.
                    Otherwise, -1 shall be returned

    \warning
*/
int TestSpi()
{
	const _SlSyncPattern_t     H2NCnysPattern = H2N_CNYS_PATTERN;

	_SlFd_t FD = 0;
	int             irqCheck = 0;
	unsigned char            pBuf[8] = {'\0'};
	unsigned char            SyncCnt  = 0;
	unsigned char            ShiftIdx = 0;

	/* Start the state machine */
	SetTestStage(TEST_STAGE_SPI_TEST_BEGIN);

	/* Initialize the Spi interface */
	FD = sl_IfOpen(0, 0);
	SetTestStage(TEST_STAGE_SPI_OPEN_PASSED);

	/* Register IRQ handler */
	sl_IfRegIntHdlr((P_EVENT_HANDLER)DiagIrqHandler, 0);

	/*Make Sure that the device is turned off */
	sl_DeviceDisable();
	SetTestStage(TEST_STAGE_DISABLE_PASSED);

	/* Save IRQ counter before activating the Device */
	irqCheck = g_irqCounter;

	/* Turn on the device */
	sl_DeviceEnable();
	SetTestStage(TEST_STAGE_ENABLE_PASSED);

	/* Wait until we get an increment on IRQ value */
	while( irqCheck == g_irqCounter )
	{
		k_sleep(100);
	}

	SetTestStage(TEST_STAGE_SPI_IRQ_PASSED);
	k_sleep(100);
	/* Generate the sync pattern for getting the response */
	sl_IfWrite(FD, (unsigned char *)&H2NCnysPattern, SYNC_PATTERN_LEN);
	k_sleep(100);

	SetTestStage(TEST_STAGE_SPI_WRITE_PASSED);



	/* Read 4 bytes from the Spi */
	sl_IfRead(FD, &pBuf[0], 4);

	/* Sync on read pattern */
	while ( ! N2H_SYNC_PATTERN_MATCH(pBuf, TxSeqNum))
	{
		k_sleep(1000);
		/* Read next 4 bytes to Low 4 bytes of buffer */
		if(0 == (SyncCnt % SYNC_PATTERN_LEN))
		{
			sl_IfRead(FD, &pBuf[4], 4);
		}

		/* Shift Buffer Up for checking if the sync is shifted */
		for(ShiftIdx = 0; ShiftIdx < 7; ShiftIdx++)
		{
			pBuf[ShiftIdx] = pBuf[ShiftIdx+1];
		}
		pBuf[7] = 0;
		SyncCnt++;
	}

	/*Sync pattern found. If needed, complete number of read bytes to multiple
	* of 4 (protocol align) */
	SyncCnt %= SYNC_PATTERN_LEN;

	if(SyncCnt > 0)
	{
		*(_u32 *)&pBuf[0] = *(_u32 *)&pBuf[4];
		sl_IfRead(FD, &pBuf[SYNC_PATTERN_LEN - SyncCnt], SyncCnt);
	}
	else
	{
		sl_IfRead(FD, &pBuf[0], 4);
	}

	/* Scan for possible double pattern */
	while( N2H_SYNC_PATTERN_MATCH(pBuf, TxSeqNum))
	{
		sl_IfRead(FD, &pBuf[0], SYNC_PATTERN_LEN);
	}

	/* Read the Resp Specific header (4 more bytes) */
	sl_IfRead(FD, &pBuf[SYNC_PATTERN_LEN], 4);
	SetTestStage(TEST_STAGE_SPI_READ_PASSED);

	/* Check the init complete message opcode */
	if(SL_OPCODE_DEVICE_INITCOMPLETE != (*(unsigned short*)(pBuf)))
	{
		LOG_DBG("Error in Spi Testing\r\n");
		return -1; /* failed to read init complete */
	}

	SetTestStage(TEST_STAGE_INIT_COMPLETE_READ_PASSED);
	SetTestStage(TEST_STAGE_SPI_TEST_COMPLETE);

	return SUCCESS;
}

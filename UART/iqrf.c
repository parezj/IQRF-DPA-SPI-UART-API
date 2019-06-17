/*
 * Name:    iqrf.c
 * Author:  Jakub Parez
 * Date:    23.1.2019
 * Version: 0.2.1
 *
 */

/*  IQRF (DPA over UART) API for IQRF OS 4.0 and higher
 *  ==================================================
 *  LEDs:    Green 1 pulse -->  init & enable OK
 *             Red 1 pulse -->  init & enable ERR
 *           Green on  ------>  FRC read in progress
 *             Red on  ------>  poll read in progress
 */

#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

#include "iqrf.h"
#include "iqrf_priv.h"

#include "iqrf_comm.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "assert.h"
#include "mem.h"

static volatile IQRF_STATE iqrf_state = IQRF_STATE_DIS;

// RTOS handles
static SemaphoreHandle_t iqrf_entry_sem;

// IQRF API - UART data
static uint8_t iqrf_uart_out_buffer[IQRF_UART_MAX_LEN];
static uint8_t iqrf_uart_in_buffer[IQRF_UART_MAX_LEN + 1];

// IQRF API - DPA data
static IQRF_NADR_L available_nodes[IQRF_MAX_NODES];
static uint8_t available_nodes_count;
static uint8_t bonded_nodes_count;
static IQRF_NADR_L node_highest_addr;
static uint8_t discovery_id;

/*-----------------------------------------------------------------------------------------------------------+
 +                                                  public                                                   +
 +-----------------------------------------------------------------------------------------------------------*/

void iqrf_init(void)
{
    iqrf_entry_sem = xSemaphoreCreateMutex();
    assert(iqrf_entry_sem != NULL);
}

uint8_t iqrf_en(void)
{
    uint8_t ret = 1;

    assert(xSemaphoreTake(iqrf_entry_sem, portMAX_DELAY) == pdPASS);

    if (iqrf_state == IQRF_STATE_DIS)
    {
    	iqrf_comm_en();
    	iqrf_state = IQRF_STATE_EN;

        // IQRF init procedure - cca 200m
        uint8_t _bonded_nodes_count = 0;
        uint8_t bonded_nodes_bitmap[IQRF_DPA_NODES_BITMAP_SIZE];
        uint8_t discovered_nodes_bitmap[IQRF_DPA_NODES_BITMAP_SIZE];
        node_highest_addr = 0;
        memset((void *) available_nodes, 0, IQRF_MAX_NODES);

        vTaskDelay(pdMS_TO_TICKS(500));

        IQRF_DPA_MESSAGE_STATUS stat1 = iqrf_dpa_get_bonded_nodes(bonded_nodes_bitmap);
        IQRF_DPA_MESSAGE_STATUS stat2 = iqrf_dpa_get_discovered_nodes(discovered_nodes_bitmap);
        IQRF_DPA_MESSAGE_STATUS stat3 = iqrf_dpa_get_addressing_information(&_bonded_nodes_count,
                                                                            &discovery_id);

        if (stat1.dpa_resp_code       == IQRF_DPA_STATUS_NO_ERROR &&
            stat2.dpa_resp_code       == IQRF_DPA_STATUS_NO_ERROR &&
            stat3.dpa_resp_code       == IQRF_DPA_STATUS_NO_ERROR)
        {
            int8_t available_nodes_result = iqrf_get_available_nodes(available_nodes,
                                                                     bonded_nodes_bitmap,
                                                                     discovered_nodes_bitmap,
                                                                     _bonded_nodes_count,
                                                                     &node_highest_addr);
            if (available_nodes_result >= 0)
            {
                bonded_nodes_count = _bonded_nodes_count;
                available_nodes_count = available_nodes_result;
                ret = 0;
            }
            else
                available_nodes_count = 0;

            iqrf_dpa_led_control(NADR_COORDINATOR_L, IQRF_LED_GREEN, IQRF_LED_PULSE);
        }
        else
        {
            iqrf_dpa_led_control(NADR_COORDINATOR_L, IQRF_LED_RED, IQRF_LED_PULSE);
        }
    }
    assert(xSemaphoreGive(iqrf_entry_sem) == pdPASS);

    return ret;
}

void iqrf_dis(void)
{
    assert(xSemaphoreTake(iqrf_entry_sem, portMAX_DELAY) == pdPASS);

    iqrf_comm_dis();
    iqrf_state = IQRF_STATE_DIS;

    assert(xSemaphoreGive(iqrf_entry_sem) == pdPASS);
}

uint8_t iqrf_rd_frc_uf(uint64_t *urgent_flags, uint64_t *battery_flags)
{
    *urgent_flags = 0ull;
    *battery_flags = 0ull;
    return 0;   // urgent flag is not implemented yet at nodes side

    uint8_t ret = 1u;

    assert(xSemaphoreTake(iqrf_entry_sem, portMAX_DELAY) == pdPASS);

    if (iqrf_state == IQRF_STATE_EN)
    {
        IQRF_DPA_MESSAGE_OUT msg1;
        IQRF_DPA_MESSAGE_OUT msg2;
        *urgent_flags = 0ull;
        *battery_flags = 0ull;

        uint32_t frc_timeout = IQRF_FRC_TIME_CALC(bonded_nodes_count, available_nodes_count);

        iqrf_dpa_led_control(NADR_COORDINATOR_L, IQRF_LED_GREEN, IQRF_LED_ON);

        msg1 = iqrf_dpa_message(IQRF_DPA_MSG_RESPONSE_ONLY,
                                frc_timeout,
                                IQRF_DPA_MSG_EX(NADR_COORDINATOR_LH,
                                                PNUM_FRC,
                                                CMD_FRC_SEND,
                                                HWPID_DONT_CHECK_LH,
                                                IQRF_PDATA_URGENT));

        if (node_highest_addr > IQRF_DPA_FRC_MAX_ADDR_2BIT)
        {
            msg2 = iqrf_dpa_message(IQRF_DPA_MSG_RESPONSE_ONLY,
                                    0,
                                    IQRF_DPA_MSG_EX(NADR_COORDINATOR_LH,
                                                    PNUM_FRC,
                                                    CMD_FRC_EXTRARESULT,
                                                    HWPID_DONT_CHECK_LH));
        }

        if ((msg1.status.dpa_resp_code  == IQRF_DPA_STATUS_NO_ERROR &&
             msg1.response.Data_len     == IQRF_DPA_FRC_MAX_LEN + 1) &&

            ((node_highest_addr         <= IQRF_DPA_FRC_MAX_ADDR_2BIT) ||

            (msg2.status.dpa_resp_code  == IQRF_DPA_STATUS_NO_ERROR &&
             msg2.response.Data_len     == IQRF_DPA_FRC_EXTRA_LEN)))
        {
            for (uint8_t i = 0; i < URGENT_FLAGS_BIT_SZ; i++)   //  0-29B -> 0-232b (1st) --> 0-64b
                *urgent_flags |= (BITMAP_GET_BIT(msg1.response.Data, i, 8) << i);

            for (uint8_t j = 0; j < BATTERY_FLAGS_BIT_SZ; j++)  // 32-55B -> 0-184b (2nd) --> 0-64b
                *battery_flags |= (BITMAP_GET_BIT(msg1.response.Data + IQRF_DPA_FRC_2BIT_2ND_START,
                                                  j,
                                                  8) << j);

            if (node_highest_addr > IQRF_DPA_FRC_MAX_ADDR_2BIT) // bits from nodes with addrs > 185
            {
                //for (uint8_t i = 0; i < IQRF_DPA_FRC_EXTRA_LEN_2BIT; i++) // 56-61B -> 185-232b (2nd)
                //    *other_flags |= (BITMAP_GET_BIT(msg2.response.Data, i, 8) << i);
            }

            ret = 0u;
        }

        iqrf_dpa_led_control(NADR_COORDINATOR_L, IQRF_LED_GREEN, IQRF_LED_OFF);
    }

    assert(xSemaphoreGive(iqrf_entry_sem) == pdPASS);

    return ret;
}

uint8_t iqrf_rd_frc_data(IQRF_FRC_DATA data[], uint64_t *valid_flags)
{
    uint8_t ret = 1u;

    assert(xSemaphoreTake(iqrf_entry_sem, portMAX_DELAY) == pdPASS);

    if (iqrf_state == IQRF_STATE_EN)
    {
        IQRF_DPA_MESSAGE_OUT chB_l_1;
        IQRF_DPA_MESSAGE_OUT chB_h_1;
        IQRF_DPA_MESSAGE_OUT chA_l_1;
        IQRF_DPA_MESSAGE_OUT chA_h_1;
        IQRF_DPA_MESSAGE_OUT chB_l_2;
        IQRF_DPA_MESSAGE_OUT chB_h_2;
        IQRF_DPA_MESSAGE_OUT chA_l_2;
        IQRF_DPA_MESSAGE_OUT chA_h_2;
        *valid_flags = 0ull;

        uint16_t sleep_seconds = 0;    // todo ??
        uint16_t sleep_scaled = sleep_seconds / IQRF_SLEEP_N_SCALER;

        iqrf_dpa_led_control(NADR_COORDINATOR_L, IQRF_LED_GREEN, IQRF_LED_ON);

        IQRF_DPA_MESSAGE_OUT bcast = iqrf_dpa_message(IQRF_DPA_MSG_CONFIRM_ONLY,
                                                      0,
                                                      IQRF_DPA_MSG_EX(NADR_BROADCAST_LH,
                                                                      PNUM_USER,
                                                                      IQRF_CUSTOM_NODE_READ,
                                                                      HWPID_DONT_CHECK_LH));

        vTaskDelay(pdMS_TO_TICKS(IQRF_DPA_TIME_CALC_EX(bcast.confirmation,
                                                       IQRF_DPA_FRC_TIMEOUT2 + IQRF_MEAS_TIMESPAN_MAX)));
        //iqrf_poll_check(IQRF_DPA_POLL_UNTIL_TIMEOUT,
                        //IQRF_DPA_POLLCHECK_INTERVAL,
                        //IQRF_DPA_TIME_CALC_EX(bcast.confirmation,
                        //                      IQRF_DPA_FRC_TIMEOUT2 + IQRF_MEAS_TIMESPAN_MAX),
                        //IQRF_FIRST_CHECK);

        uint32_t frc_timeout = IQRF_FRC_TIME_CALC(bonded_nodes_count, available_nodes_count);

        chB_l_1 = iqrf_dpa_message(IQRF_DPA_MSG_RESPONSE_ONLY,
                                   frc_timeout,
                                   IQRF_DPA_MSG_EX(NADR_COORDINATOR_LH,
                                                   PNUM_FRC,
                                                   CMD_FRC_SEND,
                                                   HWPID_DONT_CHECK_LH,
                                                   IQRF_PDATA_READ_CH_B_L));

        if (node_highest_addr > IQRF_DPA_FRC_MAX_LEN)
        {
            chB_l_2 = iqrf_dpa_message(IQRF_DPA_MSG_RESPONSE_ONLY,
                                       0,
                                       IQRF_DPA_MSG_EX(NADR_COORDINATOR_LH,
                                                       PNUM_FRC,
                                                       CMD_FRC_EXTRARESULT,
                                                       HWPID_DONT_CHECK_LH));
        }

        chB_h_1 = iqrf_dpa_message(IQRF_DPA_MSG_RESPONSE_ONLY,
                                   frc_timeout,
                                   IQRF_DPA_MSG_EX(NADR_COORDINATOR_LH,
                                                   PNUM_FRC,
                                                   CMD_FRC_SEND,
                                                   HWPID_DONT_CHECK_LH,
                                                   IQRF_PDATA_READ_CH_B_H));

        if (node_highest_addr > IQRF_DPA_FRC_MAX_LEN)
        {
            chB_h_2 = iqrf_dpa_message(IQRF_DPA_MSG_RESPONSE_ONLY,
                                       0,
                                       IQRF_DPA_MSG_EX(NADR_COORDINATOR_LH,
                                                       PNUM_FRC,
                                                       CMD_FRC_EXTRARESULT,
                                                       HWPID_DONT_CHECK_LH));
        }

        chA_l_1 = iqrf_dpa_message(IQRF_DPA_MSG_RESPONSE_ONLY,
                                   frc_timeout,
                                   IQRF_DPA_MSG_EX(NADR_COORDINATOR_LH,
                                                   PNUM_FRC,
                                                   CMD_FRC_SEND,
                                                   HWPID_DONT_CHECK_LH,
                                                   IQRF_PDATA_READ_CH_A_L));

        if (node_highest_addr > IQRF_DPA_FRC_MAX_LEN)
        {
            chA_l_2 = iqrf_dpa_message(IQRF_DPA_MSG_RESPONSE_ONLY,
                                       0,
                                       IQRF_DPA_MSG_EX(NADR_COORDINATOR_LH,
                                                       PNUM_FRC,
                                                       CMD_FRC_EXTRARESULT,
                                                       HWPID_DONT_CHECK_LH));
        }

        uint8_t last_pdata_len = IQRF_PDATA_FRC_SLEEP_LEN + IQRF_DPA_FOURSOME_LEN;
        uint8_t last_pdata[last_pdata_len];
        last_pdata[0] = NADR_COORDINATOR_L;
        last_pdata[1] = NADR_DEFAULT_H;
        last_pdata[2] = PNUM_FRC;
        last_pdata[3] = CMD_FRC_SEND;
        last_pdata[4] = HWPID_DONT_CHECK_L;
        last_pdata[5] = HWPID_DONT_CHECK_H;

        if (sleep_seconds == 0)
        {
            uint8_t last_pdata_postf[IQRF_PDATA_FRC_LEN] = { IQRF_PDATA_READ_CH_A_H };
            last_pdata_len = IQRF_PDATA_FRC_LEN + IQRF_DPA_FOURSOME_LEN;
            for (uint8_t i = IQRF_DPA_FOURSOME_LEN, j = 0; j < IQRF_PDATA_FRC_LEN; i++, j++)
                last_pdata[i] = last_pdata_postf[j];
        }
        else
        {
            last_pdata[6] = CMD_FRC_ACK_BROADCAST;
            uint8_t last_pdata_postf[IQRF_PDATA_FRC_SLEEP_LEN] =
                { IQRF_PDATA_READ_CH_A_H_AND_SLEEP(sleep_scaled) };
            for (uint8_t i = IQRF_DPA_FOURSOME_LEN + 1, j = 0; j < IQRF_PDATA_FRC_SLEEP_LEN; i++, j++)
                last_pdata[i] = last_pdata_postf[j];
        }

        chA_h_1 = iqrf_dpa_message(IQRF_DPA_MSG_RESPONSE_ONLY, frc_timeout, last_pdata, last_pdata_len);

        if (node_highest_addr > IQRF_DPA_FRC_MAX_LEN)
        {
            chA_h_2 = iqrf_dpa_message(IQRF_DPA_MSG_RESPONSE_ONLY,
                                       0,
                                       IQRF_DPA_MSG_EX(NADR_COORDINATOR_LH,
                                                       PNUM_FRC,
                                                       CMD_FRC_EXTRARESULT,
                                                       HWPID_DONT_CHECK_LH));
        }

        if ((bcast.status.dpa_resp_code == IQRF_DPA_STATUS_NO_ERROR &&

            IS_EQUAL_4_AND(IQRF_DPA_STATUS_NO_ERROR,
                           chB_l_1.status.dpa_resp_code,
                           chB_h_1.status.dpa_resp_code,
                           chA_l_1.status.dpa_resp_code,
                           chA_h_1.status.dpa_resp_code) &&
            IS_EQUAL_4_AND(IQRF_DPA_FRC_MAX_LEN + 1,
                           chB_l_1.response.Data_len,
                           chB_h_1.response.Data_len,
                           chA_l_1.response.Data_len,
                           chA_h_1.response.Data_len)) &&

           ((node_highest_addr <= IQRF_DPA_FRC_MAX_LEN) ||

           (IS_EQUAL_4_AND(IQRF_DPA_STATUS_NO_ERROR,
                           chB_l_2.status.dpa_resp_code,
                           chB_h_2.status.dpa_resp_code,
                           chA_l_2.status.dpa_resp_code,
                           chA_h_2.status.dpa_resp_code) &&
            IS_EQUAL_4_AND(IQRF_DPA_FRC_EXTRA_LEN,
                           chB_l_2.response.Data_len,
                           chB_h_2.response.Data_len,
                           chA_l_2.response.Data_len,
                           chA_h_2.response.Data_len))))
        {
            uint8_t i, k;
            for (i = 0, k = 1; i < IQRF_DPA_FRC_MAX_LEN; i++, k++)
            {
                data[i].chan_a = IQRF_FRC_DECODE(*(chA_h_1.response.Data + k),
                                                 *(chA_l_1.response.Data + k));
                data[i].chan_b = IQRF_FRC_DECODE(*(chB_h_1.response.Data + k),
                                                 *(chB_l_1.response.Data + k));

                *valid_flags |= (((data[i].chan_a == IQRF_FRC_OFFLINE_VALUE ||
                                   data[i].chan_b == IQRF_FRC_OFFLINE_VALUE) ? 0ull : 1ull) << i);
            }

            if (node_highest_addr > IQRF_DPA_FRC_MAX_LEN)
            {
                for (uint8_t j = 0; j < IQRF_DPA_FRC_EXTRA_LEN; j++, i++)
                {
                    data[i].chan_a = IQRF_FRC_DECODE(*(chA_h_2.response.Data + j),
                                                     *(chA_l_2.response.Data + j));
                    data[i].chan_b = IQRF_FRC_DECODE(*(chB_h_2.response.Data + j),
                                                     *(chB_l_2.response.Data + j));

                    *valid_flags |= (((data[i].chan_a == IQRF_FRC_OFFLINE_VALUE ||
                                       data[i].chan_b == IQRF_FRC_OFFLINE_VALUE) ? 0ull : 1ull) << i);
                }
            }

            ret = 0u;
        }

        iqrf_dpa_led_control(NADR_COORDINATOR_L, IQRF_LED_GREEN, IQRF_LED_OFF);
    }

    assert(xSemaphoreGive(iqrf_entry_sem) == pdPASS);

    return ret;
}

uint8_t iqrf_rd_poll_data(IQRF_POLL_DATA data[], uint64_t *valid_flags)
{
    uint8_t ret = 1u;

    assert(xSemaphoreTake(iqrf_entry_sem, portMAX_DELAY) == pdPASS);

    if (iqrf_state == IQRF_STATE_EN)
    {
        *valid_flags = 0ull;

        /*
         *this bcast could speed up reading X-times if node-read fnc implemented at nodes sides
        IQRF_DPA_MESSAGE_OUT bcast = iqrf_dpa_message(IQRF_DPA_MSG_CONFIRM_ONLY,
                                                      0,
                                                      IQRF_DPA_MSG_EX(NADR_BROADCAST_LH,
                                                                      PNUM_USER,
                                                                      CMD_USER_N_READ,
                                                                      HWPID_DONT_CHECK_LH));

        if (bcast.status.dpa_resp_code == IQRF_DPA_STATUS_NO_ERROR)
        {
            iqrf_poll_check(IQRF_DPA_POLL_UNTIL_TIMEOUT,
                            IQRF_DPA_POLLCHECK_INTERVAL,
                            IQRF_DPA_TIME_CALC_EX(bcast.confirmation, 0), //IQRF_MEAS_TIMESPAN_MAX
                            IQRF_FIRST_CHECK);
		}
        */
        ret = 0u;

        iqrf_dpa_led_control(NADR_COORDINATOR_L, IQRF_LED_RED, IQRF_LED_ON);

        for(uint8_t i = 0; i < available_nodes_count; i++)
        {
            IQRF_DPA_MESSAGE_OUT msg = iqrf_dpa_message(IQRF_DPA_MSG_CONF_AND_RESP,
                                                        IQRF_MEAS_TIMESPAN_MAX, // this cause delay
                                                        IQRF_DPA_MSG_EX(available_nodes[i],
                                                                        NADR_DEFAULT_H,
                                                                        PNUM_USER,
                                                                        IQRF_CUSTOM_NODE_READ,
                                                                        HWPID_DONT_CHECK_LH));

            if (msg.status.dpa_resp_code   == IQRF_DPA_STATUS_NO_ERROR &&
                msg.response.Data_len      == IQRF_POLL_DATA_LEN &&
                msg.response.foursome.NADR  < IQRF_SENS_CNT)  // only naddr < 64 because data[64]
            {
                uint8_t node_naddr = msg.response.foursome.NADR;

                if (node_naddr < VALID_FLAGS_BIT_SZ)
                    *valid_flags |= (1ull << node_naddr);

                data[node_naddr].chan_a  = CONVERT_U8_TO_U16(*(msg.response.Data + 1),
                                                             *(msg.response.Data));
                data[node_naddr].chan_b  = CONVERT_U8_TO_U16(*(msg.response.Data + 3),
                                                             *(msg.response.Data + 2));
                data[node_naddr].naddr   = msg.response.foursome.NADR;
                data[node_naddr].pnum    = msg.response.foursome.PNUM;
                data[node_naddr].pcmd    = msg.response.foursome.PCMD;
                data[node_naddr].hwpid   = msg.response.foursome.HWPID;
                data[node_naddr].errn    = msg.response.resp_code;  // 0
                data[node_naddr].dpa_val = msg.response.DPA_value;  // rssi
            }
            else if (msg.status.dpa_resp_code != IQRF_DPA_STATUS_NO_RESPONSE)
            {
                ret = 1u;
                break;
            }
        }

        iqrf_dpa_led_control(NADR_COORDINATOR_L, IQRF_LED_RED, IQRF_LED_OFF);
    }
    assert(xSemaphoreGive(iqrf_entry_sem) == pdPASS);

    return ret;
}


/*-----------------------------------------------------------------------------------------------------------+
 +                                                  private                                                  +
 +-----------------------------------------------------------------------------------------------------------*/
// ____ IQRF API :: main _____________________________________________________________________________________

// iqrf_dpa_message() is top level fnc of this API, main interface for any kind of IQRF DPA manipulation
static IQRF_DPA_MESSAGE_OUT iqrf_dpa_message(IQRF_DPA_MSG_MODE mode,
                                             uint32_t timeout,
                                             const uint8_t* msg,
                                             uint8_t len)
{
    IQRF_DPA_CONFIRMATION confirm;
    IQRF_DPA_MESSAGE_OUT out;
    uint32_t _timeout = timeout;

    if (_timeout == 0)
        _timeout = IQRF_DPA_RESPONSE_TIMEOUT;
    out.mode = mode;

    #if (IQRF_SPI_DELAY_BEFORE_MSG > 0)
        vTaskDelay(pdMS_TO_TICKS(IQRF_SPI_DELAY_BEFORE_MSG));
    #endif

    uint8_t i;
    assert(len <= IQRF_UART_MAX_LEN);
    for(i = 0; i < len; i++)
		iqrf_uart_out_buffer[i] = *(msg + i);
    iqrf_uart_out_buffer[i++] = iqrf_1wire_crc(msg, len, 0xFF);

    for (uint8_t x = 0; x < IQRF_DPA_MESSAGE_RETRIES; x++)
    {
    	IRQF_COMM_XFER_RX_DESC rx1[1];
    	rx1[0].data = iqrf_uart_in_buffer;
    	rx1[0].data_len_lim = IQRF_UART_MAX_LEN + 1;
    	uint8_t ret1 = iqrf_comm_xfer((mode == IQRF_DPA_MSG_RESPONSE_ONLY) ? _timeout :
                															IQRF_DPA_CONFIRMATION_TIMEOUT,
									  iqrf_uart_out_buffer, i,
									  rx1, 1);

    	if (ret1 != 0 || rx1[0].data_len < 2)
    	{
    		out.status.uart_result = IQRF_UART_TIMEOUT;
    		out.status.dpa_resp_code = IQRF_DPA_STATUS_NO_RESPONSE;
    		return out;
    	}
    	else if (iqrf_uart_in_buffer[rx1[0].data_len - 1] !=
    	         iqrf_1wire_crc(iqrf_uart_in_buffer, rx1[0].data_len - 1, 0xFF))
    	{
    		out.status.uart_result = IQRF_UART_BAD_CRC;
    		out.status.dpa_resp_code = IQRF_DPA_ERROR_FAIL;
    		return out;
    	}

    	IQRF_CMD_YIELD yield1;
    	yield1.result = IQRF_UART_SUCCESS;
    	yield1.data = iqrf_uart_in_buffer;
    	yield1.data_len_in_buff = rx1[0].data_len - 1;

        if (mode == IQRF_DPA_MSG_RESPONSE_ONLY)
        {
            out.response = iqrf_dpa_msg_to_response(yield1);
            out.status.uart_result = IQRF_UART_SUCCESS;
            out.status.dpa_resp_code = out.response.resp_code;
            return out;
        }
        else
        {
        	confirm = iqrf_dpa_msg_to_confirmation(yield1);
			out.confirmation = confirm;

			if (mode == IQRF_DPA_MSG_CONFIRM_ONLY)
			{
                out.status.uart_result = confirm.result;
                out.status.dpa_resp_code = IQRF_DPA_STATUS_NO_ERROR;
                return out;
			}

			//else mode == IQRF_DPA_MSG_CONF_AND_RESP

	    	IRQF_COMM_XFER_RX_DESC rx2[1];
	        rx2[0].data = iqrf_uart_in_buffer;
	        rx2[0].data_len_lim = IQRF_UART_MAX_LEN + 1;
	    	uint8_t ret2 = iqrf_comm_xfer(IQRF_DPA_TIME_CALC_EX(confirm, _timeout),
										  NULL, 0,
										  rx2, 1);

	    	if (ret2 != 0 || rx2[0].data_len < 2)
	    	{
	    		out.status.uart_result = IQRF_UART_TIMEOUT;
	    		out.status.dpa_resp_code = IQRF_DPA_STATUS_NO_RESPONSE;
	    		return out;
	    	}
	    	else if (iqrf_uart_in_buffer[rx2[0].data_len - 1] !=
	    	         iqrf_1wire_crc(iqrf_uart_in_buffer, rx2[0].data_len - 1, 0xFF))
	    	{
	    		out.status.uart_result = IQRF_UART_BAD_CRC;
	    		out.status.dpa_resp_code = IQRF_DPA_ERROR_FAIL;
	    		return out;
	    	}

	    	IQRF_CMD_YIELD yield2;
	    	yield2.result = IQRF_UART_SUCCESS;
	    	yield2.data = iqrf_uart_in_buffer;
	    	yield2.data_len_in_buff = rx2[0].data_len - 1;

            out.response = iqrf_dpa_msg_to_response(yield2);
            out.status.uart_result = IQRF_UART_SUCCESS;
            out.status.dpa_resp_code = out.response.resp_code;

        }
    }
    #if (IQRF_SPI_DELAY_AFTER_MSG > 0)
        vTaskDelay(pdMS_TO_TICKS(IQRF_SPI_DELAY_AFTER_MSG));
    #endif

    return out;
}

// friendly interface mainly for debug time - no need to count bytes
/*
static IQRF_DPA_MESSAGE_OUT iqrf_dpa_message_ex(IQRF_DPA_MESSAGE_IN msg)
{
    uint8_t arr[msg.PData_len + IQRF_DPA_FOURSOME_LEN];
    iqrf_dpa_msg_struct_to_array(msg, arr);
    return iqrf_dpa_message(msg.mode,
                            msg.timeout,
                            arr,
                            msg.PData_len + IQRF_DPA_FOURSOME_LEN);
}
*/

// ____ IQRF API :: wrappers _________________________________________________________________________________

static IQRF_DPA_MESSAGE_STATUS iqrf_dpa_get_addressing_information(uint8_t* out_DevNr, uint8_t* out_DID)
{
    *out_DevNr = 0;
    *out_DID = 0;

    IQRF_DPA_MESSAGE_OUT msg = iqrf_dpa_message(IQRF_DPA_MSG_RESPONSE_ONLY,
                                                0,
                                                IQRF_DPA_MSG_EX(NADR_COORDINATOR_LH,
                                                                PNUM_COORDINATOR,
                                                                CMD_COORDINATOR_ADDR_INFO,
                                                                HWPID_DONT_CHECK_LH));

    if (msg.status.dpa_resp_code == IQRF_DPA_STATUS_NO_ERROR &&
        msg.response.Data_len    == IQRF_DPA_ADDR_INFO_LEN)
    {
        *out_DevNr =  *msg.response.Data;
        *out_DID   = *(msg.response.Data + 1);
        return msg.status;
    }
    return (msg.status.dpa_resp_code != IQRF_DPA_STATUS_NO_ERROR) ? msg.status :
                                                                    IQRF_DPA_STATUS_WRONG_RX_LEN;
}

static IQRF_DPA_MESSAGE_STATUS iqrf_dpa_get_discovered_nodes(uint8_t* out_bitmap_32B)
{
    IQRF_DPA_MESSAGE_OUT msg = iqrf_dpa_message(IQRF_DPA_MSG_RESPONSE_ONLY,
                                                0,
                                                IQRF_DPA_MSG_EX(NADR_COORDINATOR_LH,
                                                                PNUM_COORDINATOR,
                                                                CMD_COORDINATOR_DISCOVERED_DEVICES,
                                                                HWPID_DONT_CHECK_LH));

    if (msg.status.dpa_resp_code == IQRF_DPA_STATUS_NO_ERROR &&
        msg.response.Data_len    == IQRF_DPA_NODES_BITMAP_SIZE)
    {
        for (uint8_t i = 0; i < IQRF_DPA_NODES_BITMAP_SIZE; i++)
            *(out_bitmap_32B + i) = *(msg.response.Data + i);
        return msg.status;
    }
    return (msg.status.dpa_resp_code != IQRF_DPA_STATUS_NO_ERROR) ? msg.status :
                                                                    IQRF_DPA_STATUS_WRONG_RX_LEN;
}

static IQRF_DPA_MESSAGE_STATUS iqrf_dpa_get_bonded_nodes(uint8_t* out_bitmap_32B)
{
    IQRF_DPA_MESSAGE_OUT msg = iqrf_dpa_message(IQRF_DPA_MSG_RESPONSE_ONLY,
                                                0,
                                                IQRF_DPA_MSG_EX(NADR_COORDINATOR_LH,
                                                                PNUM_COORDINATOR,
                                                                CMD_COORDINATOR_BONDED_DEVICES,
                                                                HWPID_DONT_CHECK_LH));

    if (msg.status.dpa_resp_code == IQRF_DPA_STATUS_NO_ERROR &&
        msg.response.Data_len    == IQRF_DPA_NODES_BITMAP_SIZE)
    {
        for (uint8_t i = 0; i < IQRF_DPA_NODES_BITMAP_SIZE; i++)
            *(out_bitmap_32B + i) = *(msg.response.Data + i);
        return msg.status;
    }
    return (msg.status.dpa_resp_code != IQRF_DPA_STATUS_NO_ERROR) ? msg.status :
                                                                    IQRF_DPA_STATUS_WRONG_RX_LEN;
}

static IQRF_DPA_MESSAGE_STATUS iqrf_dpa_led_control(IQRF_NADR_L nadr, IQRF_LED led, IQRF_LED_ACTION action)
{
    return iqrf_dpa_message(IQRF_DPA_MSG_RESPONSE_ONLY,
                            0,
                            IQRF_DPA_MSG_EX(nadr,
                                            NADR_DEFAULT_H,
                                            led,
                                            action,
                                            HWPID_DONT_CHECK_LH)).status;
}


static IQRF_DPA_DISCOVERY_INFO iqrf_dpa_discovery(uint8_t tx_power, uint8_t max_addr, uint32_t timeout)
{
    IQRF_DPA_MESSAGE_OUT msg = iqrf_dpa_message(IQRF_DPA_MSG_RESPONSE_ONLY,
                                                timeout,
                                                IQRF_DPA_MSG_EX(NADR_COORDINATOR_LH,
                                                                PNUM_COORDINATOR,
                                                                CMD_COORDINATOR_DISCOVERY,
                                                                HWPID_DONT_CHECK_LH,
                                                                tx_power,
                                                                max_addr));
    IQRF_DPA_DISCOVERY_INFO ret;
    ret.status = msg.status;
    if (msg.status.dpa_resp_code == IQRF_DPA_STATUS_NO_ERROR &&
        msg.response.Data_len    == IQRF_DPA_DISCOVERY_INFO_LEN)
    {
        ret.disc_nr = *(msg.response.Data);
        return ret;
    }
    ret.status = (msg.status.dpa_resp_code != IQRF_DPA_STATUS_NO_ERROR) ? msg.status :
                                                                          IQRF_DPA_STATUS_WRONG_RX_LEN;
    return ret;
}

static IQRF_DPA_DISCOVERY_DATA iqrf_dpa_discovery_data()
{
    IQRF_DPA_DISCOVERY_DATA ret;
    uint8_t batchs = IQRF_DPA_DISCOVERY_DATA_LEN / IQRF_DPA_DISCOVERY_DATA_NOD;
    uint16_t addr_vrns    = IQRF_EEEPROM_ADDR_VRNS;
    uint16_t addr_zones   = IQRF_EEEPROM_ADDR_ZONES;
    uint16_t addr_parents = IQRF_EEEPROM_ADDR_PARENTS;

    for(int i = 0; i < batchs; i++)
    {
        IQRF_DPA_MESSAGE_OUT msg_vrns = iqrf_dpa_message(IQRF_DPA_MSG_RESPONSE_ONLY,
                                                         0,
                                                         IQRF_DPA_MSG_EX(NADR_COORDINATOR_LH,
                                                                         PNUM_COORDINATOR,
                                                                         CMD_COORDINATOR_DISCOVERY_DATA,
                                                                         HWPID_DONT_CHECK_LH,
                                                                         LO_BYTE16(addr_vrns),
                                                                         HI_BYTE16(addr_vrns)));

        IQRF_DPA_MESSAGE_OUT msg_zones = iqrf_dpa_message(IQRF_DPA_MSG_RESPONSE_ONLY,
                                                          0,
                                                          IQRF_DPA_MSG_EX(NADR_COORDINATOR_LH,
                                                                          PNUM_COORDINATOR,
                                                                          CMD_COORDINATOR_DISCOVERY_DATA,
                                                                          HWPID_DONT_CHECK_LH,
                                                                          LO_BYTE16(addr_zones),
                                                                          HI_BYTE16(addr_zones)));

        IQRF_DPA_MESSAGE_OUT msg_parents = iqrf_dpa_message(IQRF_DPA_MSG_RESPONSE_ONLY,
                                                            0,
                                                            IQRF_DPA_MSG_EX(NADR_COORDINATOR_LH,
                                                                            PNUM_COORDINATOR,
                                                                            CMD_COORDINATOR_DISCOVERY_DATA,
                                                                            HWPID_DONT_CHECK_LH,
                                                                            LO_BYTE16(addr_parents),
                                                                            HI_BYTE16(addr_parents)));

        addr_vrns    += IQRF_DPA_DISCOVERY_DATA_NOD;
        addr_zones   += IQRF_DPA_DISCOVERY_DATA_NOD;
        addr_parents += IQRF_DPA_DISCOVERY_DATA_NOD;

        if (IS_EQUAL_3_AND(IQRF_DPA_STATUS_NO_ERROR,
                           msg_vrns.status.dpa_resp_code,
                           msg_zones.status.dpa_resp_code,
                           msg_parents.status.dpa_resp_code) &&
            IS_EQUAL_3_AND(IQRF_DPA_DISCOVERY_DATA_NOD,
                           msg_vrns.response.Data_len,
                           msg_zones.response.Data_len,
                           msg_parents.response.Data_len))
        {
            uint8_t addr_from = i * IQRF_DPA_DISCOVERY_DATA_NOD;
            for(int j = addr_from; i < addr_from + IQRF_DPA_DISCOVERY_DATA_NOD; i++)
            {
                ret.VRNs[j]    = *(msg_vrns.response.Data + i);
                ret.Zones[j]   = *(msg_zones.response.Data + i);
                ret.Parents[j] = *(msg_parents.response.Data + i);
            }
            ret.status = msg_vrns.status;
        }
        else
        {
            ret.status = (msg_vrns.status.dpa_resp_code != IQRF_DPA_STATUS_NO_ERROR) ?
                    msg_vrns.status : IQRF_DPA_STATUS_WRONG_RX_LEN;
            return ret;
        }
    }
    return ret;
}

static IQRF_DPA_NODE_INFO iqrf_dpa_node_read(IQRF_NADR_L nadr)
{
    IQRF_DPA_MESSAGE_OUT msg = iqrf_dpa_message(IQRF_DPA_MSG_RESPONSE_ONLY,
                                                0,
                                                IQRF_DPA_MSG_EX(nadr,
                                                                NADR_DEFAULT_H,
                                                                PNUM_NODE,
                                                                CMD_NODE_READ,
                                                                HWPID_DONT_CHECK_LH));
    IQRF_DPA_NODE_INFO ret;
    ret.status = msg.status;
    if (msg.status.dpa_resp_code == IQRF_DPA_STATUS_NO_ERROR &&
        msg.response.Data_len    == IQRF_DPA_NODE_INFO_LEN)
    {
        ret = iqrf_dpa_data_to_node_info(msg.response.Data);
        return ret;
    }
    ret.status = (msg.status.dpa_resp_code != IQRF_DPA_STATUS_NO_ERROR) ? msg.status :
                                                                          IQRF_DPA_STATUS_WRONG_RX_LEN;
    return ret;
}

static IQRF_DPA_OS_INFO iqrf_dpa_os_read(IQRF_NADR_L nadr)
{
    IQRF_DPA_MESSAGE_OUT msg = iqrf_dpa_message(IQRF_DPA_MSG_RESPONSE_ONLY,
                                                0,
                                                IQRF_DPA_MSG_EX(nadr,
                                                                NADR_DEFAULT_H,
                                                                PNUM_OS,
                                                                CMD_OS_READ,
                                                                HWPID_DONT_CHECK_LH));
    IQRF_DPA_OS_INFO ret;
    ret.status = msg.status;
    if (msg.status.dpa_resp_code == IQRF_DPA_STATUS_NO_ERROR &&
        msg.response.Data_len    == IQRF_DPA_OS_INFO_LEN)
    {
        ret = iqrf_dpa_data_to_os_info(msg.response.Data);
        return ret;
    }
    ret.status = (msg.status.dpa_resp_code != IQRF_DPA_STATUS_NO_ERROR) ? msg.status :
                                                                          IQRF_DPA_STATUS_WRONG_RX_LEN;
    return ret;
}

static IQRF_DPA_MESSAGE_STATUS iqrf_dpa_os_reset(IQRF_NADR_L nadr)
{
    return iqrf_dpa_message(IQRF_DPA_MSG_RESPONSE_ONLY,
                            0,
                            IQRF_DPA_MSG_EX(nadr,
                                            NADR_DEFAULT_H,
                                            PNUM_OS,
                                            CMD_OS_RESET,
                                            HWPID_DONT_CHECK_LH)).status;
}

static IQRF_DPA_PERIPHERAL_ENUM iqrf_dpa_peripheral_enumeration(IQRF_NADR_L nadr)
{
    IQRF_DPA_MESSAGE_OUT msg = iqrf_dpa_message(IQRF_DPA_MSG_RESPONSE_ONLY,
                                                0,
                                                IQRF_DPA_MSG_EX(nadr,
                                                                NADR_DEFAULT_H,
                                                                PNUM_ENUM,
                                                                CMD_GET_PER_INFO,
                                                                HWPID_DONT_CHECK_LH));
    IQRF_DPA_PERIPHERAL_ENUM ret;
    ret.status = msg.status;
    if (msg.status.dpa_resp_code == IQRF_DPA_STATUS_NO_ERROR &&
        msg.response.Data_len >= IQRF_DPA_PER_ENUM_MIN_LEN)
    {
        ret = iqrf_dpa_data_to_per_enum(msg.response.Data, msg.response.Data_len);
        return ret;
    }
    ret.status = (msg.status.dpa_resp_code != IQRF_DPA_STATUS_NO_ERROR) ? msg.status :
                                                                          IQRF_DPA_STATUS_WRONG_RX_LEN;
    return ret;
}

static IQRF_DPA_MESSAGE_STATUS iqrf_dpa_backup(IQRF_NADR_L nadr, uint8_t** out_data_49B, uint8_t* out_len)
{
    uint8_t len = 0;
    uint8_t last_byte = 0;
    IQRF_DPA_MESSAGE_STATUS ret;
    IQRF_PNUM pnum = PNUM_COORDINATOR;
    IQRF_PCMD pcmd = CMD_COORDINATOR_BACKUP;

    if (nadr != NADR_COORDINATOR_L)
    {
        pnum = PNUM_NODE;
        pcmd = CMD_NODE_BACKUP;
    }

    do
    {
        IQRF_DPA_MESSAGE_OUT msg = iqrf_dpa_message(IQRF_DPA_MSG_RESPONSE_ONLY,
                                                    0,
                                                    IQRF_DPA_MSG_EX(nadr,
                                                                    NADR_DEFAULT_H,
                                                                    pnum,
                                                                    pcmd,
                                                                    HWPID_DONT_CHECK_LH,
                                                                    len));

        if (msg.status.dpa_resp_code == IQRF_DPA_STATUS_NO_ERROR &&
            msg.response.Data_len    >  0)
        {
            last_byte = *(msg.response.Data + msg.response.Data_len - 1);
            ret = msg.status;
            for (uint8_t i = 0; i < msg.response.Data_len; i++)
                *((uint8_t*)out_data_49B + (IQRF_DPA_BACKUP_NODE_LEN * len) + i) = *(msg.response.Data + i);
            len++;
        }
        else
        {
            *out_len = 0;
            return (msg.status.dpa_resp_code != IQRF_DPA_STATUS_NO_ERROR) ? msg.status :
                                                                            IQRF_DPA_STATUS_WRONG_RX_LEN;
        }
    }
    while(last_byte > 0);

    *out_len = len;
    return ret;
}

static IQRF_DPA_MESSAGE_STATUS iqrf_dpa_restore(IQRF_NADR_L nadr, uint8_t** data_49B, uint8_t len)
{
    assert(len > 0);
    IQRF_DPA_MESSAGE_STATUS ret;
    IQRF_PNUM pnum = PNUM_COORDINATOR;
    IQRF_PCMD pcmd = CMD_COORDINATOR_RESTORE;
    uint8_t dat_len = IQRF_DPA_BACKUP_NODE_LEN + IQRF_DPA_FOURSOME_LEN;

    if (nadr != NADR_COORDINATOR_L)
    {
        pnum = PNUM_NODE;
        pcmd = CMD_NODE_RESTORE;
    }

    for(int i = 0; i < len; i++)
    {
        uint8_t dat[dat_len];
        dat[0] = nadr;
        dat[1] = NADR_DEFAULT_H;
        dat[2] = pnum;
        dat[3] = pcmd;
        dat[4] = HWPID_DONT_CHECK_L;
        dat[5] = HWPID_DONT_CHECK_H;

        for(int j = IQRF_DPA_FOURSOME_LEN, k = 0; j < dat_len; j++, k++)
            dat[j] = *((uint8_t*)data_49B + (IQRF_DPA_BACKUP_NODE_LEN * len) + k);

        IQRF_DPA_MESSAGE_OUT msg = iqrf_dpa_message(IQRF_DPA_MSG_RESPONSE_ONLY,
                                                    0,
                                                    dat,
                                                    dat_len);
        ret = msg.status;
        if (msg.status.dpa_resp_code != IQRF_DPA_STATUS_NO_ERROR)
            return ret;
    }
    return ret;
}

static IQRF_DPA_MESSAGE_STATUS iqrf_dpa_clear_all_bonds()
{
    return iqrf_dpa_message(IQRF_DPA_MSG_RESPONSE_ONLY,
                            0,
                            IQRF_DPA_MSG_EX(NADR_COORDINATOR_LH,
                                            PNUM_COORDINATOR,
                                            CMD_COORDINATOR_CLEAR_ALL_BONDS,
                                            HWPID_DONT_CHECK_LH)).status;
    //ensure to rescan available nodes
}

static IQRF_DPA_MESSAGE_STATUS iqrf_dpa_remove_bonded_node(uint8_t BondAddr, uint8_t* out_DevNr)
{
    *out_DevNr = 0;

    IQRF_DPA_MESSAGE_OUT msg = iqrf_dpa_message(IQRF_DPA_MSG_RESPONSE_ONLY,
                                                0,
                                                IQRF_DPA_MSG_EX(NADR_COORDINATOR_LH,
                                                                PNUM_COORDINATOR,
                                                                CMD_COORDINATOR_BOND_NODE,
                                                                HWPID_DONT_CHECK_LH,
                                                                BondAddr));
    if (msg.status.dpa_resp_code == IQRF_DPA_STATUS_NO_ERROR &&
        msg.response.Data_len    == IQRF_DPA_REMOVE_BOND_LEN)
    {
        *out_DevNr =  *msg.response.Data;
        return msg.status;
    }
    return (msg.status.dpa_resp_code != IQRF_DPA_STATUS_NO_ERROR) ? msg.status :
                                                                    IQRF_DPA_STATUS_WRONG_RX_LEN;
    //ensure to rescan available nodes
}

static IQRF_DPA_MESSAGE_STATUS iqrf_dpa_bond_node(uint8_t* out_BondAddr, uint8_t* out_DevNr)
{
    return iqrf_dpa_bond_node_ex(0, 0, out_BondAddr, out_DevNr);
}

static IQRF_DPA_MESSAGE_STATUS iqrf_dpa_bond_node_ex(uint8_t ReqAddr,
                                                      uint8_t mask,
                                                      uint8_t* out_BondAddr,
                                                      uint8_t* out_DevNr)
{
    *out_BondAddr = 0;
    *out_DevNr = 0;

    IQRF_DPA_MESSAGE_OUT msg = iqrf_dpa_message(IQRF_DPA_MSG_RESPONSE_ONLY,
                                                IQRF_DPA_BOND_TIMEOUT,
                                                IQRF_DPA_MSG_EX(NADR_COORDINATOR_LH,
                                                                PNUM_COORDINATOR,
                                                                CMD_COORDINATOR_BOND_NODE,
                                                                HWPID_DONT_CHECK_LH,
                                                                ReqAddr,
                                                                mask));
    if (msg.status.dpa_resp_code == IQRF_DPA_STATUS_NO_ERROR &&
        msg.response.Data_len    == IQRF_DPA_BOND_NODE_LEN)
    {
        *out_BondAddr =  *msg.response.Data;
        *out_DevNr    = *(msg.response.Data + 1);
        return msg.status;
    }
    return (msg.status.dpa_resp_code != IQRF_DPA_STATUS_NO_ERROR) ? msg.status :
                                                                    IQRF_DPA_STATUS_WRONG_RX_LEN;
    //ensure to rescan available nodes
}


// ____ IQRF API :: core utils  ______________________________________________________________________________

inline static IQRF_DPA_CONFIRMATION iqrf_dpa_msg_to_confirmation(IQRF_CMD_YIELD yield)
{
    IQRF_DPA_CONFIRMATION ret;
    if (yield.data_len_in_buff == IQRF_DPA_CONFIRMATION_LEN)
    {
        ret.result    =  yield.result;
        ret.foursome  = iqrf_dpa_msg_to_foursome(yield);
        ret.resp_code = *(yield.data + 6);
        ret.DPA_value = *(yield.data + 7);
        ret.hops_out  = *(yield.data + 8);
        ret.timeslot  = *(yield.data + 9);
        ret.hops_back = *(yield.data + 10);
    }
    else
    {
        ret.result    = IQRF_UART_GENERAL_ERR;
        ret.foursome  = IQRF_DPA_FOURSOME_EMPTY;
        ret.resp_code = IQRF_DPA_ERROR_FAIL;
        ret.DPA_value = 0;
        ret.hops_out  = 0;
        ret.timeslot  = 0;
        ret.hops_back = 0;
    }
    return ret;
}

inline static IQRF_DPA_NOTIFICATION iqrf_dpa_msg_to_notification(IQRF_CMD_YIELD yield)
{
    IQRF_DPA_NOTIFICATION ret;
    if (yield.data_len_in_buff == IQRF_DPA_NOTIFICATION_LEN)
    {
        ret.result   = yield.result;
        ret.foursome = iqrf_dpa_msg_to_foursome(yield);
    }
    else ret.result = IQRF_UART_GENERAL_ERR;

    return ret;
}

inline static IQRF_DPA_RESPONSE iqrf_dpa_msg_to_response(IQRF_CMD_YIELD yield)
{
    IQRF_DPA_RESPONSE ret;
    if (yield.data_len_in_buff >= IQRF_DPA_DEFAULT_LEN)
    {
        uint8_t i;
        ret.result    =  yield.result;
        ret.foursome  = iqrf_dpa_msg_to_foursome(yield);
        ret.resp_code = *(yield.data + 6);
        ret.DPA_value = *(yield.data + 7);
        for (i = 0; i < yield.data_len_in_buff - IQRF_DPA_FOURSOME_LEN - 2; i++)
            ret.Data[i] = *(yield.data + IQRF_DPA_FOURSOME_LEN + i + 2);
        ret.Data_len = i;
    }
    else
    {
        ret.result    = IQRF_UART_GENERAL_ERR;
        ret.foursome  = IQRF_DPA_FOURSOME_EMPTY;
        ret.resp_code = IQRF_DPA_ERROR_FAIL;
        ret.DPA_value = 0;
        ret.Data_len  = 0;
    }
    return ret;
}

inline static IQRF_DPA_FOURSOME iqrf_dpa_msg_to_foursome(IQRF_CMD_YIELD yield)
{
    IQRF_DPA_FOURSOME ret;
    ret.NADR  = CONVERT_U8_TO_U16(
                *(yield.data + 1),
                *(yield.data    ));
    ret.PNUM  = *(yield.data + 2);
    ret.PCMD  = *(yield.data + 3);
    ret.HWPID = CONVERT_U8_TO_U16(
                *(yield.data + 5),
                *(yield.data + 4));
    return ret;
}

inline static uint8_t iqrf_1wire_crc(const uint8_t* data, uint8_t len, uint8_t initial)
{
    // The 1-Wire CRC scheme is described in Maxim Application Note 27:
    // "Understanding and Using Cyclic Redundancy Checks with Maxim iButton Products"
     uint8_t crc = initial;

     for (uint8_t i = 0; i < len; i++)
     {
           uint8_t inbyte = data[i];
           for (uint8_t j = 0; j < 8; j++)
           {
                 uint8_t mix = (crc ^ inbyte) & 0x01;
                 crc >>= 1;
                 if (mix)
                       crc ^= 0x8C;

                 inbyte >>= 1;
           }
     }
     return crc;
}

// returns -1 if there is an error, otherwise sets "available_nodes" and returns node count
inline static int8_t iqrf_get_available_nodes(uint8_t* available_nodes,
                                              uint8_t* bonded_nodes_bitmap,
                                              uint8_t* discovered_nodes_bitmap,
                                              uint8_t bonded_nodes_count,
                                              IQRF_NADR_L* node_highest_addr)
{
    uint8_t bonded_counter = 0;
    int8_t available_counter = 0;
    IQRF_NADR_L _node_highest_addr = 0;

    for(uint8_t i = NADR_COORDINATOR_L; i < NADR_BROADCAST_L; i++)
    {
        if (BITMAP_GET_BIT(bonded_nodes_bitmap, i, 8))
        {
            bonded_counter++;
            if (BITMAP_GET_BIT(discovered_nodes_bitmap, i, 8))
            {
                *(available_nodes + available_counter) = i;
                if (i > _node_highest_addr)
                    _node_highest_addr = i;
                available_counter++;
            }
        }
    }

    if(bonded_counter == bonded_nodes_count)
    {
        *node_highest_addr = _node_highest_addr;
        return available_counter;
    }
    else
        return -1;
}


// ____ IQRF API :: wrappers utils ___________________________________________________________________________

inline static void iqrf_dpa_msg_struct_to_array(IQRF_DPA_MESSAGE_IN msg, uint8_t* array)
{
    //uint8_t *array = malloc(msg.PData_len + IQRF_DPA_FOURSOME_LEN);
    *array       = HI_BYTE16(msg.foursome.NADR);
    *(array + 1) = LO_BYTE16(msg.foursome.NADR);
    *(array + 2) = msg.foursome.PNUM;
    *(array + 3) = msg.foursome.PCMD;
    *(array + 4) = HI_BYTE16(msg.foursome.HWPID);
    *(array + 5) = LO_BYTE16(msg.foursome.HWPID);
    for (uint8_t i = 0; i < msg.PData_len; i++)
        *(array + 5 + i) = msg.PData[i];
    //return array;
}

inline static IQRF_DPA_NODE_INFO iqrf_dpa_data_to_node_info(uint8_t* data)
{
    IQRF_DPA_NODE_INFO ret;
    ret.ntw_addr        = *(data);
    ret.ntw_vrn         = *(data +  1);
    ret.ntw_zin         = *(data +  2);
    ret.ntw_did         = *(data +  3);
    ret.ntw_pvrn        = *(data +  4);
    ret.ntw_useraddress = CONVERT_U8_TO_U16(
                          *(data +  6),
                          *(data +  5));
    ret.ntw_id          = CONVERT_U8_TO_U16(
                          *(data +  8),
                          *(data +  7));
    ret.ntw_vrnfnz      = *(data +  9);
    ret.ntw_cfg         = *(data + 10);
    ret.flags           = *(data + 11);
    return ret;
}

inline static IQRF_DPA_OS_INFO iqrf_dpa_data_to_os_info(uint8_t* data)
{
    IQRF_DPA_OS_INFO ret;
    ret.module_id      = CONVERT_U8_TO_U32(
                         *(data),
                         *(data +  1),
                         *(data +  2),
                         *(data +  3));
    ret.os_version     = *(data +  4);
    ret.mcu_type       = *(data +  5);
    ret.os_build       = CONVERT_U8_TO_U16(
                         *(data +  7),
                         *(data +  6));
    ret.rssi           = *(data +  8);
    ret.supply_voltage = *(data +  9);
    ret.flags          = *(data + 10);
    ret.slot_limits    = *(data + 11);
    return ret;
}


// ____ custom tools _________________________________________________________________________________________

/*
static IQRF_DPA_MESSAGE_STATUS iqrf_coordinator_put_sleep(uint16_t seconds)
{
    uint16_t sleep_scaled = seconds /= IQRF_SLEEP_C_SCALER;

    return iqrf_dpa_message(IQRF_DPA_MSG_RESPONSE_ONLY,
                            0,
                            IQRF_DPA_MSG_EX(NADR_COORDINATOR_LH,
                                            PNUM_USER,
                                            IQRF_CUSTOM_COORD_SLEEP,
                                            HWPID_DONT_CHECK_LH,
                                            LO_BYTE16(sleep_scaled),
                                            HI_BYTE16(sleep_scaled),)).status;
}

static IQRF_DPA_MESSAGE_STATUS iqrf_node_put_sleep(IQRF_NADR_L nadr, uint16_t seconds)
{
    uint8_t i, j = 0;
    uint16_t sleep_scaled = seconds /= IQRF_SLEEP_N_SCALER;
    uint32_t frc_timeout = IQRF_FRC_TIME_CALC(bonded_nodes_count, available_nodes_count);
    uint8_t selected_nodes[IQRF_DPA_SELECTED_NODES_LEN] = { 0 };
    uint8_t sleep_pdata[IQRF_PDATA_FRC_SLEEP_LEN] = { IQRF_PDATA_READ_CH_A_H_AND_SLEEP(sleep_scaled) };
    uint8_t msg_len = IQRF_DPA_FOURSOME_LEN + IQRF_DPA_SELECTED_NODES_LEN + 1 + IQRF_PDATA_FRC_SLEEP_LEN;
    uint8_t msg[msg_len];

    msg[0] = NADR_COORDINATOR_L;
    msg[1] = NADR_DEFAULT_H;
    msg[2] = PNUM_FRC;
    msg[3] = CMD_FRC_SEND_SELECTIVE;
    msg[4] = HWPID_DONT_CHECK_L;
    msg[5] = HWPID_DONT_CHECK_H;
    msg[6] = CMD_FRC_ACK_BROADCAST;

    BITMAP_SET_BIT(selected_nodes, nadr, 8);

    for (i = IQRF_DPA_FOURSOME_LEN + 1, j = 0; j < IQRF_DPA_SELECTED_NODES_LEN; i++, j++)
        msg[i] = selected_nodes[j];

    for (j = 0; j < IQRF_PDATA_FRC_SLEEP_LEN; i++, j++)
        msg[i] = sleep_pdata[j];

    return iqrf_dpa_message(IQRF_DPA_MSG_RESPONSE_ONLY, frc_timeout, msg, msg_len).status;
}

static IQRF_DPA_MESSAGE_STATUS iqrf_all_nodes_put_sleep(uint16_t seconds)
{
    uint16_t sleep_scaled = seconds /= IQRF_SLEEP_N_SCALER;
    uint32_t frc_timeout = IQRF_FRC_TIME_CALC(bonded_nodes_count, available_nodes_count);

    return iqrf_dpa_message(IQRF_DPA_MSG_RESPONSE_ONLY,
                            frc_timeout,
                            IQRF_DPA_MSG_EX(NADR_COORDINATOR_LH,
                                            PNUM_FRC,
                                            CMD_FRC_SEND,
                                            HWPID_DONT_CHECK_LH,
                                            CMD_FRC_ACK_BROADCAST,
                                            IQRF_PDATA_READ_CH_A_H_AND_SLEEP(sleep_scaled))).status;
}
*/

// ____ test ________________________________________________________________________________________________

static uint16_t test_iters;
static float temp;

void iqrf_test()
{
    IQRF_POLL_DATA data_poll[IQRF_SENS_CNT];
    IQRF_FRC_DATA data_frc[IQRF_SENS_CNT];

    uint64_t valid_flags_poll = 0ull;
    uint64_t valid_flags_frc = 0ull;
    //uint64_t urgent_flags = 0ull;
    //uint64_t battery_flags = 0ull;

    // infinite iqrf test, get flags + data by poll and frc, and compare results
    while(1) // cca 11s = 1 iter
    {
        /*____ POLL read
         * doba trvani: 3s => [n nodu] * (IQRF_MEAS_TIMESPAN_MAX + routing time) -> lin. zvysuje s n nodu  */
        vTaskDelay(pdMS_TO_TICKS(10));
        iqrf_rd_poll_data(data_poll, &valid_flags_poll);

        /*____ FRC read
         * doba trvani: 7s + IQRF_MEAS_TIMESPAN_MAX [5 nodu] -> doba stoupa s mnozstvim nodu podle vzorce:
         * (bonded * 30) + ((discovered + 2) * 100) + 40 + 210 = ms                                        */
        vTaskDelay(pdMS_TO_TICKS(50));
        iqrf_rd_frc_data(data_frc, &valid_flags_frc);

        /*____ FRC urgent flags
        vTaskDelay(pdMS_TO_TICKS(10));
        iqrf_rd_frc_uf(&urgent_flags);  // ,&battery_flags                                                 */

        uint8_t error = 0;
        if (valid_flags_poll == valid_flags_frc)
        {
            for(uint8_t i = 0; i < IQRF_SENS_CNT; i++)
            {
                if (valid_flags_poll & (uint64_t)(1ull << i)) // node online
                {
                    if ((i == 0x02 ||  // temp at node 0x02 is variable data..
                        data_poll[i].chan_a == data_frc[i].chan_a) &&
                        data_poll[i].chan_b == data_frc[i].chan_b)
                    {
                        if (i == 0x02) // calc temp from node at addr: 0x02
                        {
                            temp = ((float)(data_poll[0x02].chan_a >> 4)) * 0.0625;
                        }
                    }
                    else
                    {
                        error++;
                    }
                }
            }
        }
        if (error != 0)
            break;

        test_iters++;
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    while (1)
    {
        //iqrf_packet_check(1);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

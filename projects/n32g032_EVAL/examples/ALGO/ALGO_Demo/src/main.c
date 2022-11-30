/*****************************************************************************
 * Copyright (c) 2019, Nations Technologies Inc.
 *
 * All rights reserved.
 * ****************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Nations' name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY NATIONS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL NATIONS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
 * @file main.c
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "n32g032.h"
#include "n32g032_rcc.h"

#include "n32g032_rng.h"
#include "n32g032_aes.h"
#include "n32g032_sm4.h"

#include "log.h"

#include <string.h>
#ifdef __IAR_ARM
#pragma pack(4)
#endif
void DumpWords(const uint32_t* words, uint32_t len)
{
    for (uint32_t i = 0; i < len; ++i)
    {
        log_info("0x%08x, ", words[i]);
    }
}

void DumpBytes(const uint8_t* bytes, uint32_t len)
{
    for (uint32_t i = 0; i < len; ++i)
    {
        log_info("%02x", bytes[i]);
    }
}

void TestRand(void)
{
    uint32_t buf[8];
    uint32_t seed[2] = {1, 2};
    if (RNG_OK != GetPseudoRand_U32(buf, 8, seed))
    {
        log_error("GetPseudoRand_U32 failed.\n");
    }
    else
    {
        log_info("Pseudo random with seed 1,2: ");
        DumpWords(buf, 8);
        log_info("\n");
    }
    seed[0] = 3;
    seed[1] = 4;
    if (RNG_OK != GetPseudoRand_U32(buf, 8, seed))
    {
        log_error("GetPseudoRand_U32 failed.\n");
    }
    else
    {
        log_info("Pseudo random with seed 3,4: ");
        DumpWords(buf, 8);
        log_info("\n");
    }

    if (RNG_OK != GetTrueRand_U32(buf, 8))
    {
        log_error("GetTrueRand_U32 failed.\n");
    }
    else
    {
        log_info("True random: ");
        DumpWords(buf, 8);
        log_info("\n");
    }
}

void TestAES(void)
{
    AES_PARM AES_Parm;
#ifdef __IAR_ARM
    uint8_t key[16]   = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    uint8_t plain[16] = {
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00};
    uint8_t cipher[16];
    uint8_t plainOut[16];
#else
    __align(4) uint8_t key[16]   = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    __align(4) uint8_t plain[16] = {
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00};
    __align(4) uint8_t cipher[16];
    __align(4) uint8_t plainOut[16];
#endif
    AES_Parm.in         = (uint32_t*)plain;
    AES_Parm.out        = (uint32_t*)cipher;
    AES_Parm.key        = (uint32_t*)key;
    AES_Parm.iv         = NULL; // IV is not needed in ECB mode
    AES_Parm.inWordLen  = sizeof(plain) / sizeof(uint32_t);
    AES_Parm.keyWordLen = sizeof(key) / sizeof(uint32_t);
    AES_Parm.Mode       = AES_ECB;
    AES_Parm.En_De      = AES_ENC;

    // encrypt data
    if (AES_Init_OK != AES_Init(&AES_Parm))
    {
        log_error("AES_Init failed.\n");
        return;
    }
    if (AES_Crypto_OK != AES_Crypto(&AES_Parm))
    {
        log_error("AES_Crypto failed\n");
        return;
    }

    log_info("AES ECB encrypt:\n");
    log_info("key = ");
    DumpBytes(key, sizeof(key));
    log_info("\n");
    log_info("plain = ");
    DumpBytes(plain, sizeof(plain));
    log_info("\n");
    log_info("cipher = ");
    DumpBytes(cipher, sizeof(cipher));
    log_info("\n");

    AES_Parm.in    = (uint32_t*)cipher;
    AES_Parm.out   = (uint32_t*)plainOut;
    AES_Parm.En_De = AES_DEC;

    // decrypt data
    if (AES_Init_OK != AES_Init(&AES_Parm))
    {
        log_error("AES_Init failed.\n");
        return;
    }
    if (AES_Crypto_OK != AES_Crypto(&AES_Parm))
    {
        log_error("AES_Crypto failed\n");
        return;
    }

    log_info("decrypt out = ");
    DumpBytes(plainOut, sizeof(plainOut));
    log_info("\n");
    if (memcmp(plain, plainOut, sizeof(plain)) != 0)
    {
        log_error("AES decrypt result do not equal plain data.\n");
        return;
    }
}

void TestSM4(void)
{
    SM4_PARM SM4_Parm;
#ifdef __IAR_ARM
    uint8_t key[16]   = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    uint8_t plain[16] = {
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00};
    uint8_t cipher[16];
    uint8_t plainOut[16];
#else
    __align(4) uint8_t key[16]   = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    __align(4) uint8_t plain[16] = {
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00};
    __align(4) uint8_t cipher[16];
    __align(4) uint8_t plainOut[16];
#endif
    SM4_Parm.in        = (uint32_t*)plain;
    SM4_Parm.out       = (uint32_t*)cipher;
    SM4_Parm.key       = (uint32_t*)key;
    SM4_Parm.iv        = NULL; // IV is not needed in ECB mode
    SM4_Parm.inWordLen = sizeof(plain) / sizeof(uint32_t);
    SM4_Parm.workingMode = SM4_ECB;
    SM4_Parm.EnDeMode  = SM4_ENC;

    // encrypt data
    if (SM4_Init_OK != SM4_Init(&SM4_Parm))
    {
        log_error("SM4_Init failed.\n");
        return;
    }
    if (SM4_Crypto_OK != SM4_Crypto(&SM4_Parm))
    {
        log_error("SM4_Crypto failed\n");
        return;
    }

    log_info("SM4 ECB encrypt:\n");
    log_info("key = ");
    DumpBytes(key, sizeof(key));
    log_info("\n");
    log_info("plain = ");
    DumpBytes(plain, sizeof(plain));
    log_info("\n");
    log_info("cipher = ");
    DumpBytes(cipher, sizeof(cipher));
    log_info("\n");

    SM4_Parm.in    = (uint32_t*)cipher;
    SM4_Parm.out   = (uint32_t*)plainOut;
    SM4_Parm.EnDeMode = SM4_DEC;

    // decrypt data
    if (SM4_Init_OK != SM4_Init(&SM4_Parm))
    {
        log_error("SM4_Init failed.\n");
        return;
    }
    if (SM4_Crypto_OK != SM4_Crypto(&SM4_Parm))
    {
        log_error("SM4_Crypto failed\n");
        return;
    }

    log_info("decrypt out = ");
    DumpBytes(plainOut, 16);
    log_info("\n");
    if (memcmp(plain, plainOut, sizeof(plain)) != 0)
    {
        log_error("SM4 decrypt result do not equal plain data.\n");
        return;
    }
}

int main(void)
{
    log_init();
    log_info("-----------------------\nAlgorithm demo start.\n");

    // RNG test
    TestRand();

    // Cryptogram algorithm    
    TestAES();
    TestSM4();

    while (1)
        ;
}

#ifdef USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param file pointer to the source file name
 * @param line assert_param error line source number
 */
void assert_failed(const uint8_t* expr, const uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}

#endif

/**
 ******************************************************************************
 * @file    main.c
 * @author  WIZnet
 * @author  WEN-LIANG LIN modify 20240706
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2018 WIZnet</center></h2>
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "wizchip_conf.h"
#include "dhcp.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/** @addtogroup W7500x_StdPeriph_Examples
 * @{
 */

/** @addtogroup Empty
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DATA_BUF_SIZE 2048

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static __IO uint32_t TimingDelay;
uint8_t test_buf[DATA_BUF_SIZE];
wiz_NetInfo gWIZNETINFO;

/* Private function prototypes -----------------------------------------------*/
static void User_LED_GPIO_Config(void);
void Delay(__IO uint32_t nTime);
static void Lock_ON(void);
static void Lock_OPEN(void);
uint8_t Lock_state=0; //0 - open, 1 - lock

static void UART_Config(void);
static void GPIO_Config(void);
static void DUALTIMER_Config(void);
static void Network_Config(void);
void dhcp_assign(void);
void dhcp_update(void);
void dhcp_conflict(void);
int32_t WebServer(uint8_t sn, uint8_t *buf, uint16_t port);
void delay(__IO uint32_t milliseconds);
void TimingDelay_Decrement(void);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main program.
 * @param  None
 * @retval None
 */
int main(void)
{
    uint32_t ret;
    uint8_t dhcp_retry = 0;
    SystemInit();
    /* SysTick_Config */
    SysTick_Config((GetSystemClock() / 1000));

    /* Set WZ_100US Register */
    setTIC100US((GetSystemClock() / 10000));

    UART_Config();
    GPIO_Config();
    DUALTIMER_Config();

    PWM_DeInit(PWM5);
    Lock_ON();

    User_LED_GPIO_Config();

    printf("W7500x Standard Peripheral Library version : %d.%d.%d\r\n", __W7500X_STDPERIPH_VERSION_MAIN, __W7500X_STDPERIPH_VERSION_SUB1, __W7500X_STDPERIPH_VERSION_SUB2);

    printf("SourceClock : %d\r\n", (int)GetSourceClock());
    printf("SystemClock : %d\r\n", (int)GetSystemClock());

    /* Initialize PHY */
#ifdef W7500
    printf("PHY Init : %s\r\n", PHY_Init(GPIOB, GPIO_Pin_15, GPIO_Pin_14) == SET ? "Success" : "Fail");
#elif defined(W7500P)
    printf("PHY Init : %s\r\n", PHY_Init(GPIOB, GPIO_Pin_14, GPIO_Pin_15) == SET ? "Success" : "Fail");
#endif

    /* Check Link */
    printf("Link : %s\r\n", PHY_GetLinkStatus() == PHY_LINK_ON ? "On" : "Off");

    /* Network information setting. Here use STATIC IP. */
    Network_Config();

    printf("System Loop Start\r\n");

    while (1)
    {
        WebServer(1, test_buf, 80);
    }

    //    return 0;
}

#ifdef USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif

/**
 * @}
 */

/**
 * @brief  Configures the PWM Peripheral.
 * @note
 * @param  None
 * @retval None
 */

// https://github.com/arduino-libraries/Servo/blob/master/src/Servo.h
#define MIN_PULSE_WIDTH 544      // the shortest pulse sent to a servo
#define MAX_PULSE_WIDTH 2400     // the longest pulse sent to a servo
#define DEFAULT_PULSE_WIDTH 1500 // default pulse width when servo is attached
#define REFRESH_INTERVAL 20000   // minimum time to refresh servos in microseconds

static void Lock_ON(void)
{
    PWM_InitTypeDef PWM_InitStructure;

    PWM_Cmd(PWM5, DISABLE);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource5, PAD_AF2);

    PWM_StructInit(&PWM_InitStructure);

    PWM_InitStructure.PWM_Output = PWM_Output_OutEnable_InDisable;
    // 1/8,000,000Hz = 0.000000125s
    // 0.000000125 * 8 = 0.000001s
    PWM_InitStructure.PWM_Prescale_Counter = 8 - 1;
    PWM_InitStructure.PWM_Duty = DEFAULT_PULSE_WIDTH - 1;
    PWM_InitStructure.PWM_Period = REFRESH_INTERVAL - 1;
    PWM_Init(PWM5, &PWM_InitStructure);
    PWM_Cmd(PWM5, ENABLE);

    Lock_state=1;
}

static void Lock_OPEN(void)
{
    PWM_InitTypeDef PWM_InitStructure;

    PWM_Cmd(PWM5, DISABLE);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource5, PAD_AF2);

    PWM_StructInit(&PWM_InitStructure);

    PWM_InitStructure.PWM_Output = PWM_Output_OutEnable_InDisable;
    // 1/8,000,000Hz = 0.000000125s
    // 0.000000125 * 8 = 0.000001s
    PWM_InitStructure.PWM_Prescale_Counter = 8 - 1;
    PWM_InitStructure.PWM_Duty = MIN_PULSE_WIDTH - 1;
    PWM_InitStructure.PWM_Period = REFRESH_INTERVAL - 1;
    PWM_Init(PWM5, &PWM_InitStructure);
    PWM_Cmd(PWM5, ENABLE);

    Lock_state=0;
}

/**
 * @brief  Configures the GPIO Peripheral.
 * @note
 * @param  None
 * @retval None
 */
static void User_LED_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Direction = GPIO_Direction_OUT;
    GPIO_InitStructure.GPIO_AF = PAD_AF1;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/**
 * @brief  Configures the UART Peripheral.
 * @note
 * @param  None
 * @retval None
 */
static void UART_Config(void)
{
    UART_InitTypeDef UART_InitStructure;

    UART_StructInit(&UART_InitStructure);

#if defined(USE_WIZWIKI_W7500_EVAL)
    UART_Init(UART1, &UART_InitStructure);
    UART_Cmd(UART1, ENABLE);
#else
    S_UART_Init(115200);
    S_UART_Cmd(ENABLE);
#endif
}

/**
 * @brief  Configures the GPIO Peripheral.
 * @note   GPIO pin configures for ADC
 * @param  None
 * @retval None
 */
static void GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Direction = GPIO_Direction_IN;
    GPIO_InitStructure.GPIO_Pad = GPIO_Pad_Default;
    GPIO_InitStructure.GPIO_AF = PAD_AF0;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/**
 * @brief  Configures the DUALTIMER Peripheral.
 * @note
 * @param  None
 * @retval None
 */
static void DUALTIMER_Config(void)
{
    DUALTIMER_InitTypDef DUALTIMER_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    DUALTIMER_InitStructure.Timer_Load = GetSystemClock() / 1; // 1s
    DUALTIMER_InitStructure.Timer_Prescaler = DUALTIMER_Prescaler_1;
    DUALTIMER_InitStructure.Timer_Wrapping = DUALTIMER_Periodic;
    DUALTIMER_InitStructure.Timer_Repetition = DUALTIMER_Wrapping;
    DUALTIMER_InitStructure.Timer_Size = DUALTIMER_Size_32;
    DUALTIMER_Init(DUALTIMER0_0, &DUALTIMER_InitStructure);

    DUALTIMER_ITConfig(DUALTIMER0_0, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = DUALTIMER0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    DUALTIMER_Cmd(DUALTIMER0_0, ENABLE);
}

/**
 * @brief  Configures the Network Information.
 * @note
 * @param  None
 * @retval None
 */
static void Network_Config(void)
{
    uint8_t mac_addr[6] = {0x00, 0x08, 0xDC, 0x01, 0x02, 0x03};
    uint8_t ip_addr[4] = {192, 168, 137, 200}; // the IP address is dependent on your network
    uint8_t dns_addr[4] = {192, 168, 137, 1};  // the dns server ip
    uint8_t gw_addr[4] = {192, 168, 137, 1};   // the Gateway ip
    uint8_t sn_addr[4] = {255, 255, 255, 0};   // the subnet:

    memcpy(gWIZNETINFO.mac, mac_addr, 6);
    memcpy(gWIZNETINFO.ip, ip_addr, 4);
    memcpy(gWIZNETINFO.sn, sn_addr, 4);
    memcpy(gWIZNETINFO.gw, gw_addr, 4);
    memcpy(gWIZNETINFO.dns, dns_addr, 4);

    gWIZNETINFO.dhcp = NETINFO_STATIC;

    ctlnetwork(CN_SET_NETINFO, (void *)&gWIZNETINFO);

    printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n", gWIZNETINFO.mac[0], gWIZNETINFO.mac[1], gWIZNETINFO.mac[2], gWIZNETINFO.mac[3], gWIZNETINFO.mac[4], gWIZNETINFO.mac[5]);
    printf("IP: %d.%d.%d.%d\r\n", gWIZNETINFO.ip[0], gWIZNETINFO.ip[1], gWIZNETINFO.ip[2], gWIZNETINFO.ip[3]);
    printf("GW: %d.%d.%d.%d\r\n", gWIZNETINFO.gw[0], gWIZNETINFO.gw[1], gWIZNETINFO.gw[2], gWIZNETINFO.gw[3]);
    printf("SN: %d.%d.%d.%d\r\n", gWIZNETINFO.sn[0], gWIZNETINFO.sn[1], gWIZNETINFO.sn[2], gWIZNETINFO.sn[3]);
    printf("DNS: %d.%d.%d.%d\r\n", gWIZNETINFO.dns[0], gWIZNETINFO.dns[1], gWIZNETINFO.dns[2], gWIZNETINFO.dns[3]);
}

/**
 * @brief  The call back function of ip assign.
 * @note
 * @param  None
 * @retval None
 */
void dhcp_assign(void)
{
    getIPfromDHCP(gWIZNETINFO.ip);
    getGWfromDHCP(gWIZNETINFO.gw);
    getSNfromDHCP(gWIZNETINFO.sn);
    getDNSfromDHCP(gWIZNETINFO.dns);

    ctlnetwork(CN_SET_NETINFO, (void *)&gWIZNETINFO);
}

/**
 * @brief  The call back function of ip update.
 * @note
 * @param  None
 * @retval None
 */
void dhcp_update(void)
{
    ;
}

/**
 * @brief  The call back function of ip conflict.
 * @note
 * @param  None
 * @retval None
 */
void dhcp_conflict(void)
{
    ;
}

/**
 * @brief  WebServer example function.
 * @note
 * @param  sn: Socket number to use.
 * @param  buf: The buffer the socket will use.
 * @param  port: Socket port number to use.
 * @retval Success or Fail of configuration functions
 */
int32_t WebServer(uint8_t sn, uint8_t *buf, uint16_t port)
{
    uint8_t i;
    uint8_t adcChannelOffset = 2;
    int32_t ret;
    uint16_t size = 0;
    uint8_t destip[4];
    uint16_t destport;
    uint8_t adc_buf[128] = {
        '\0',
    };
    const char* strGET_H = "GET /H";
    const char* strGET_L = "GET /L";

    switch (getSn_SR(sn))
    {
    case SOCK_ESTABLISHED:

        if (getSn_IR(sn) & Sn_IR_CON)
        {

            getSn_DIPR(sn, destip);
            destport = getSn_DPORT(sn);
            printf("%d:Connected - %d.%d.%d.%d : %d\r\n", sn, destip[0], destip[1], destip[2], destip[3], destport);

            setSn_IR(sn, Sn_IR_CON);
        }

        if ((size = getSn_RX_RSR(sn)) > 0)
        {
            if (size > DATA_BUF_SIZE)
                size = DATA_BUF_SIZE;
            ret = recv(sn, buf, size);
            if (ret <= 0)
                return ret;
            if (!strncmp(buf, strGET_H, 6))
            {
                if(!Lock_state)
                {
                    Lock_ON();
                    GPIO_ResetBits(GPIOC, GPIO_Pin_15); //user led
                }
            }
            if (!strncmp(buf, strGET_L, 6))
            {
                if(Lock_state)
                {
                    Lock_OPEN();
                    GPIO_SetBits(GPIOC, GPIO_Pin_15); //user led
                }
            }

            printf("%s", buf);

            ret = send(sn, "HTTP/1.1 200 OK\r\n"
                           "Content-Type: text/html\r\n"
                           "Connection: close\r\n"
                           "Refresh: 5\r\n"
                           "\r\n"
                           "<!DOCTYPE HTML>\r\n"
                           "<html>\r\n",
                       sizeof("HTTP/1.1 200 OK\r\n"
                              "Content-Type: text/html\r\n"
                              "Connection: close\r\n"
                              "Refresh: 5\r\n"
                              "\r\n"
                              "<!DOCTYPE HTML>\r\n"
                              "<html>\r\n") -
                           1);
            if (ret < 0)
            {
                close(sn);
                return ret;
            }

            sprintf(adc_buf, "Click <a href=\"/H\">here</a> turn the Lock on<br>\r\n");
            ret = send(sn, adc_buf, strlen(adc_buf));
            if (ret < 0)
            {
                close(sn);
                return ret;
            }

            sprintf(adc_buf, "Click <a href=\"/L\">here</a> turn the Lock open<br>\r\n");
            ret = send(sn, adc_buf, strlen(adc_buf));
            if (ret < 0)
            {
                close(sn);
                return ret;
            }
            ret = send(sn, "</html>\r\n", sizeof("</html>\r\n") - 1);
            if (ret < 0)
            {
                close(sn);
                return ret;
            }

            disconnect(sn);
        }

        break;
    case SOCK_CLOSE_WAIT:

        if ((ret = disconnect(sn)) != SOCK_OK)
            return ret;

        printf("%d:Socket Closed\r\n", sn);

        break;
    case SOCK_INIT:

        printf("%d:Listen, Web server, port [%d]\r\n", sn, port);

        if ((ret = listen(sn)) != SOCK_OK)
            return ret;

        break;
    case SOCK_CLOSED:

        if ((ret = socket(sn, Sn_MR_TCP, port, 0x00)) != sn)
            return ret;

        break;
    default:
        break;
    }
    return 1;
}

/**
 * @brief  Inserts a delay time.
 * @param  nTime: specifies the delay time length, in milliseconds.
 * @retval None
 */
void delay(__IO uint32_t milliseconds)
{
    TimingDelay = milliseconds;

    while (TimingDelay != 0)
        ;
}

/**
 * @brief  Decrements the TimingDelay variable.
 * @param  None
 * @retval None
 */
void TimingDelay_Decrement(void)
{
    if (TimingDelay != 0x00)
    {
        TimingDelay--;
    }
}

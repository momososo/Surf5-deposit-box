# add_subdirectory(ioLibrary)

#SYSTEM W7500x
add_library(SYSTEM_W7500X_FILES STATIC)

target_sources(SYSTEM_W7500X_FILES PUBLIC
        ${CMAKE_SOURCE_DIR}/Libraries/CMSIS/Device/WIZnet/W7500/Source/system_W7500x.c
        )

target_include_directories(SYSTEM_W7500X_FILES PUBLIC
        ${CMAKE_SOURCE_DIR}/Projects/W7500x_StdPeriph_Examples/GPIO/GPIO_IOToggle
        ${CMAKE_SOURCE_DIR}/Libraries/CMSIS/Device/WIZnet/W7500/Include
        ${CMAKE_SOURCE_DIR}/Libraries/W7500x_StdPeriph_Driver/inc
        ${CMAKE_SOURCE_DIR}/Libraries/CMSIS/Include
        )

#GPIO
add_library(GPIO_FILES STATIC)

target_sources(GPIO_FILES PUBLIC
        ${CMAKE_SOURCE_DIR}/Libraries/W7500x_StdPeriph_Driver/src/w7500x_gpio.c
        )

target_include_directories(GPIO_FILES PUBLIC
        ${CMAKE_SOURCE_DIR}/Libraries/W7500x_stdPeriph_Driver/inc
        )
target_link_libraries(GPIO_FILES PUBLIC
        SYSTEM_W7500X_FILES
        )

#UART
add_library(UART_FILES STATIC)

target_sources(UART_FILES PUBLIC
        ${CMAKE_SOURCE_DIR}/Libraries/W7500x_stdPeriph_Driver/src/W7500x_uart.c
        )

target_include_directories(UART_FILES PUBLIC
        ${CMAKE_SOURCE_DIR}/Libraries/W7500x_stdPeriph_Driver/inc
        )

target_link_libraries(UART_FILES PUBLIC
        GPIO_FILES
        SYSTEM_W7500X_FILES
        )


#ADC_FILES
add_library(ADC_FILES STATIC)

target_sources(ADC_FILES PUBLIC
        ${CMAKE_SOURCE_DIR}/Libraries/W7500x_stdPeriph_Driver/src/w7500x_adc.c
        )

target_include_directories(ADC_FILES PUBLIC
        ${CMAKE_SOURCE_DIR}/Libraries/W7500x_stdPeriph_Driver/inc
        )

target_link_libraries(ADC_FILES PUBLIC
        SYSTEM_W7500X_FILES
        )

#CRG_FILES
add_library(CRG_FILES STATIC)

target_sources(CRG_FILES PUBLIC
${CMAKE_SOURCE_DIR}/Libraries/W7500x_stdPeriph_Driver/src/w7500x_crg.c
        )

target_include_directories(CRG_FILES PUBLIC
${CMAKE_SOURCE_DIR}/Libraries/W7500x_stdPeriph_Driver/inc
        )

target_link_libraries(CRG_FILES PUBLIC
        SYSTEM_W7500X_FILES
        )

#DUALTIMER_FILES
add_library(DUALTIMER_FILES STATIC)

target_sources(DUALTIMER_FILES PUBLIC
${CMAKE_SOURCE_DIR}/Libraries/W7500x_stdPeriph_Driver/src/w7500x_dualtimer.c
        )

target_include_directories(DUALTIMER_FILES PUBLIC
${CMAKE_SOURCE_DIR}/Libraries/W7500x_stdPeriph_Driver/inc
        )

target_link_libraries(DUALTIMER_FILES PUBLIC
        SYSTEM_W7500X_FILES
        CRG_FILES
        )

#EXTI_FILES
add_library(EXTI_FILES STATIC)

target_sources(EXTI_FILES PUBLIC
${CMAKE_SOURCE_DIR}/Libraries/W7500x_stdPeriph_Driver/src/w7500x_exti.c
        )

target_include_directories(EXTI_FILES PUBLIC
${CMAKE_SOURCE_DIR}/Libraries/W7500x_stdPeriph_Driver/inc
        )

target_link_libraries(EXTI_FILES PUBLIC
        SYSTEM_W7500X_FILES
        GPIO_FILES
        )

#FLASH_FILES
add_library(FLASH_FILES STATIC)

target_sources(FLASH_FILES PUBLIC
${CMAKE_SOURCE_DIR}/Libraries/W7500x_stdPeriph_Driver/src/w7500x_flash.c
        )

target_include_directories(FLASH_FILES PUBLIC
${CMAKE_SOURCE_DIR}/Libraries/W7500x_stdPeriph_Driver/inc
        )

target_link_libraries(FLASH_FILES PUBLIC
        SYSTEM_W7500X_FILES
        )
        
#MIIM_FILES
add_library(MIIM_FILES STATIC)

target_sources(MIIM_FILES PUBLIC
${CMAKE_SOURCE_DIR}/Libraries/W7500x_stdPeriph_Driver/src/w7500x_miim.c
        )

target_include_directories(MIIM_FILES PUBLIC
${CMAKE_SOURCE_DIR}/Libraries/W7500x_stdPeriph_Driver/inc
        )

target_link_libraries(MIIM_FILES PUBLIC
        SYSTEM_W7500X_FILES
        GPIO_FILES
        )

#MISC_FILES
add_library(MISC_FILES STATIC)

target_sources(MISC_FILES PUBLIC
${CMAKE_SOURCE_DIR}/Libraries/W7500x_stdPeriph_Driver/src/w7500x_misc.c
        )

target_include_directories(MISC_FILES PUBLIC
${CMAKE_SOURCE_DIR}/Libraries/W7500x_stdPeriph_Driver/inc
        )

target_link_libraries(MISC_FILES PUBLIC
        SYSTEM_W7500X_FILES
        )

#PWM_FILES
add_library(PWM_FILES STATIC)

target_sources(PWM_FILES PUBLIC
        ${CMAKE_SOURCE_DIR}/Libraries/W7500x_stdPeriph_Driver/src/w7500x_pwm.c
        )

target_include_directories(PWM_FILES PUBLIC
        ${CMAKE_SOURCE_DIR}/Libraries/W7500x_stdPeriph_Driver/inc
        )

target_link_libraries(PWM_FILES PUBLIC
        SYSTEM_W7500X_FILES
        CRG_FILES
        )

#RNG_FILES
add_library(RNG_FILES STATIC)

target_sources(RNG_FILES PUBLIC
        ${CMAKE_SOURCE_DIR}/Libraries/W7500x_stdPeriph_Driver/src/w7500x_rng.c
        )

target_include_directories(RNG_FILES PUBLIC
        ${CMAKE_SOURCE_DIR}/Libraries/W7500x_stdPeriph_Driver/inc
        )

target_link_libraries(RNG_FILES PUBLIC
        SYSTEM_W7500X_FILES
        )

#RTC_FILES
add_library(RTC_FILES STATIC)

target_sources(RTC_FILES PUBLIC
        ${CMAKE_SOURCE_DIR}/Libraries/W7500x_stdPeriph_Driver/src/w7500x_rtc.c
        )

target_include_directories(RTC_FILES PUBLIC
        ${CMAKE_SOURCE_DIR}/Libraries/W7500x_stdPeriph_Driver/inc
        )

target_link_libraries(RTC_FILES PUBLIC
        SYSTEM_W7500X_FILES
        )

#SSP_FILES
add_library(SSP_FILES STATIC)

target_sources(SSP_FILES PUBLIC
        ${CMAKE_SOURCE_DIR}/Libraries/W7500x_stdPeriph_Driver/src/w7500x_ssp.c
        )

target_include_directories(SSP_FILES PUBLIC
        ${CMAKE_SOURCE_DIR}/Libraries/W7500x_stdPeriph_Driver/inc
        )

target_link_libraries(SSP_FILES PUBLIC
        SYSTEM_W7500X_FILES
        )

#WDT_FILES
add_library(WDT_FILES STATIC)

target_sources(WDT_FILES PUBLIC
        ${CMAKE_SOURCE_DIR}/Libraries/W7500x_stdPeriph_Driver/src/w7500x_wdt.c
        )

target_include_directories(WDT_FILES PUBLIC
        ${CMAKE_SOURCE_DIR}/Libraries/W7500x_stdPeriph_Driver/inc
        )

target_link_libraries(WDT_FILES PUBLIC
        SYSTEM_W7500X_FILES
        )


#WZTOE_FILES
add_library(WZTOE_FILES STATIC)

target_sources(WZTOE_FILES PUBLIC
        ${CMAKE_SOURCE_DIR}/Libraries/W7500x_stdPeriph_Driver/src/w7500x_wztoe.c
        )

target_include_directories(WZTOE_FILES PUBLIC
        ${CMAKE_SOURCE_DIR}/Libraries/W7500x_stdPeriph_Driver/inc
        )

target_link_libraries(WZTOE_FILES PUBLIC
        SYSTEM_W7500X_FILES
        )

#LOOPBACK_FILES
add_library(LOOPBACK_FILES STATIC)

target_sources(LOOPBACK_FILES PUBLIC
        ${CMAKE_SOURCE_DIR}/Libraries/ioLibrary/Application/loopback/loopback.c
        )

target_include_directories(LOOPBACK_FILES PUBLIC
        ${CMAKE_SOURCE_DIR}/Libraries/ioLibrary/Application/loopback
        )

target_link_libraries(LOOPBACK_FILES PUBLIC
        SOCKET_FILES    
        WIZCHIP_CONF_FILES
        WZTOE_FILES
        )


# SOCKET_FILES
add_library(SOCKET_FILES STATIC)

target_sources(SOCKET_FILES PUBLIC
        ${CMAKE_SOURCE_DIR}/Libraries/ioLibrary/Ethernet/socket.c
        )

target_include_directories(SOCKET_FILES PUBLIC
        ${CMAKE_SOURCE_DIR}/Libraries/ioLibrary/Ethernet
        ${CMAKE_SOURCE_DIR}/Libraries/CMSIS/Device/WIZnet/W7500/Include
        )

target_link_libraries(SOCKET_FILES PUBLIC
        SYSTEM_W7500X_FILES
        WZTOE_FILES
        )

#WIZCHIP_CONF_FILES
add_library(WIZCHIP_CONF_FILES STATIC)

target_sources(WIZCHIP_CONF_FILES PUBLIC
        ${CMAKE_SOURCE_DIR}/Libraries/ioLibrary/Ethernet/wizchip_conf.c
        )

target_include_directories(WIZCHIP_CONF_FILES PUBLIC
        ${CMAKE_SOURCE_DIR}/Libraries/ioLibrary/Ethernet
        ${CMAKE_SOURCE_DIR}/Libraries/CMSIS/Device/WIZnet/W7500/Include
        )

target_link_libraries(WIZCHIP_CONF_FILES PUBLIC
        SYSTEM_W7500X_FILES
        WZTOE_FILES
        )

#DHCP_FILES
add_library(DHCP_FILES STATIC)

target_sources(DHCP_FILES PUBLIC
        ${CMAKE_SOURCE_DIR}/Libraries/ioLibrary/Internet/DHCP/dhcp.c
        )

target_include_directories(DHCP_FILES PUBLIC
        ${CMAKE_SOURCE_DIR}/Libraries/ioLibrary/Internet/DHCP
        )

target_link_libraries(DHCP_FILES PUBLIC
        SOCKET_FILES
        WZTOE_FILES
        )

#DNS_FILES
add_library(DNS_FILES STATIC)

target_sources(DNS_FILES PUBLIC
        ${CMAKE_SOURCE_DIR}/Libraries/ioLibrary/Internet/DNS/dns.c
        )

target_include_directories(DNS_FILES PUBLIC
        ${CMAKE_SOURCE_DIR}/Libraries/ioLibrary/Internet/DNS
        )

target_link_libraries(DNS_FILES PUBLIC
        SOCKET_FILES
        WZTOE_FILES
        )

#SNTP_FILES
add_library(SNTP_FILES STATIC)

target_sources(SNTP_FILES PUBLIC
        ${CMAKE_SOURCE_DIR}/Libraries/ioLibrary/Internet/SNTP/sntp.c
        )

target_include_directories(SNTP_FILES PUBLIC
        ${CMAKE_SOURCE_DIR}/Libraries/ioLibrary/Internet/SNTP
        )

target_link_libraries(SNTP_FILES PUBLIC
        SOCKET_FILES
        WZTOE_FILES
        )


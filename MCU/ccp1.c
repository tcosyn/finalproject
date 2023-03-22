#include <xc.h>

extern uint8_t left_freq_1, left_freq_2;
static volatile uint16_t prev_value;

void CCP1_Init(void) {
	CCP1PPS = 0x10;   //RC0->CCP1:CCP1;
	CCP1CON = 0x85;     
	CCPTMRS0bits.C1TSEL = 0x1;
    PIR4bits.CCP1IF = 0;
    prev_value = 0;
}

void CCP1_ISR(void) {
    PIR4bits.CCP1IF = 0;
    uint16_t temp = prev_value;
    uint8_t freq_1, freq_2;
    freq_1 = CCPR1H;
    freq_2 = CCPR1L;
    uint16_t freq_comb = (freq_1 << 8) | freq_2;
    if (freq_comb < prev_value) {
        prev_value = freq_comb;
        uint16_t result = (0xFFFF - temp) + freq_comb;
        left_freq_1 = (result & 0xFF00) >> 8;
        left_freq_2 = (result & 0x00FF);
    } else {
        prev_value = freq_comb;
        uint16_t result = freq_comb - temp;
        left_freq_1 = (result & 0xFF00) >> 8;
        left_freq_2 = result & 0x00FF;
    }
}

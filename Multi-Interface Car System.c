/*
보고 해석하기 편하도록 문장구조 수정 및 주석 추가함
*/

#include "S32K144.h"

// PCC register
#define PCC_BASE	(0x40065000)   // PCC 기본 주소
#define PCC_PORTA	*((volatile unsigned*)(PCC_BASE + 0x124))   // port a 기본 주소
#define PCC_PORTB	*((volatile unsigned*)(PCC_BASE + 0x128))   // port b 기본 주소
#define PCC_PORTC	*((volatile unsigned*)(PCC_BASE + 0x12C))   // port c 기본 주소
#define PCC_PORTD	*((volatile unsigned*)(PCC_BASE + 0x130))   // port d 기본 주소
#define PCC_PORTE	*((volatile unsigned*)(PCC_BASE + 0x134))   // port e 기본 주소
#define PCC_LPIT	*((volatile unsigned*)(PCC_BASE + 0xDC))    // 타이머 지정을 위한 기본 주소
#define PCC_ADC0	*((volatile unsigned*)(PCC_BASE + 0xEC))    // adc 지정을 위한 기본 주소
#define PCC_FTM2    *((volatile unsigned*)(PCC_BASE + 0xE8))    // ftm2 지정을 위한 기본 주소
#define PCC_LPUART1 *((volatile unsigned*)(PCC_BASE + 0x1AC))   // UART 이용을 위한 기본 주소

#define CGC_BIT		30  // 클럭 게이트 제어 비트
#define PCS_BITS	24  // 프리클럭 선택 비트

// adc0 register
#define ADC0_BASE	(0x4003B000)   // ADC0 기본 주소
#define ADC0_SC1A	*((volatile unsigned*)(ADC0_BASE + 0x0))    // ADC 채널 선택 및 변환 시작 레지스터
#define ADC0_CFG1	*((volatile unsigned*)(ADC0_BASE + 0x40))   // ADC 구성 레지스터 1
#define ADC0_CFG2	*((volatile unsigned*)(ADC0_BASE + 0x44))   // ADC 구성 레지스터 2
#define ADC0_RA		*((volatile unsigned*)(ADC0_BASE + 0x48))   // ADC 데이터 레지스터
#define ADC0_SC2	*((volatile unsigned*)(ADC0_BASE + 0x90))   // ADC 상태 및 제어 레지스터 2
#define ADC0_SC3	*((volatile unsigned*)(ADC0_BASE + 0x94))   // ADC 상태 및 제어 레지스터 3
#define ADCH_BITS	0   // ADC 채널 선택 비트 위치
#define COCO_BIT	7   // 변환 완료 플래그 비트 위치
#define MODE_BITS	2   // 변환 모드 비트 위치
#define ADIV_BITS	5   // 클럭 분주 비트 위치
#define SMPLTS_BITS	0   // 샘플링 시간 비트 위치
#define ADTRG_BIT	6   // 트리거 선택 비트 위치
#define ADC0_SE12	12  // ADC 채널 12

// FTM0 register
#define FTM2_BASE    (0x4003A000)   // FTM2 기본 주소
#define FTM2_SC      *((volatile unsigned*)(FTM2_BASE + 0x0))    // FTM2 상태 및 제어 레지스터
#define FTM2_MOD     *((volatile unsigned*)(FTM2_BASE + 0x8))    // FTM2 모듈 레지스터
#define FTM2_C0SC    *((volatile unsigned*)(FTM2_BASE + 0xC))    // FTM2 채널 0 상태 및 제어 레지스터
#define FTM2_C0V     *((volatile unsigned*)(FTM2_BASE + 0x10))   // FTM2 채널 0 값 레지스터 (DC_IN 1)
#define FTM2_CNTIN   *((volatile unsigned*)(FTM2_BASE + 0x4C))   // FTM2 카운터 초기값 레지스터
#define FTM2_C1SC    *((volatile unsigned*)(FTM2_BASE + 0x14))   // FTM2 채널 1 상태 및 제어 레지스터
#define FTM2_C1V     *((volatile unsigned*)(FTM2_BASE + 0x18))   // FTM2 채널 1 값 레지스터 (DC_IN 2)

#define PWMEN0_BIT  16  // 채널 0 PWM 활성화 비트 위치
#define PWMEN1_BIT  17  // 채널 1 PWM 활성화 비트 위치
#define CLKS_BITS   3   // 클럭 선택 비트 위치
#define PS_BITS     0   // 프리스케일러 비트 위치
#define MSB_BIT     5   // 모드 선택 비트 위치
#define MSA_BIT     4   // 모드 선택 비트 위치
#define ELSB_BIT    3   // 엣지 선택 비트 위치
#define ELSA_BIT    2   // 엣지 선택 비트 위치

//uart register
#define LPUART1_BASE    (0x4006B000)   // LPUART1 기본 주소
#define LPUART1_BAUD    *((volatile unsigned*)(LPUART1_BASE + 0x10))   // LPUART1 보율 레지스터
#define LPUART1_STAT    *((volatile unsigned*)(LPUART1_BASE + 0x14))   // LPUART1 상태 레지스터
#define LPUART1_CTRL    *((volatile unsigned*)(LPUART1_BASE + 0x18))   // LPUART1 제어 레지스터
#define LPUART1_DATA    *((volatile unsigned*)(LPUART1_BASE + 0x1C))   // LPUART1 데이터 레지스터
#define M10_BIT    29   // 10비트 모드 비트 위치
#define OSR_BITS   24   // 오버샘플링 비트 위치
#define SBNS_BIT   13   // 스톱 비트 선택 비트 위치
#define SBR_BITS   0    // 보율 분주 비트 위치
#define TDRE_BIT   23   // 전송 데이터 레지스터 비어 있음 비트 위치
#define RDRF_BIT   21   // 수신 데이터 레지스터 비어 있지 않음 비트 위치
#define TE_BIT     19   // 전송 활성화 비트 위치
#define RE_BIT     18   // 수신 활성화 비트 위치
#define M7_BIT     11   // 7비트 모드 비트 위치
#define M_BIT      4    // 모드 비트 위치
#define PE_BIT     1    // 패리티 비트 위치

// GPIO register
#define GPIOA_BASE (0x400FF000)   // GPIOA 기본 주소
#define GPIOB_BASE (0x400FF040)   // GPIOB 기본 주소
#define GPIOC_BASE (0x400FF080)   // GPIOC 기본 주소
#define GPIOD_BASE (0x400FF0C0)   // GPIOD 기본 주소
#define GPIOE_BASE (0x400FF100)   // GPIOE 기본 주소

#define GPIOA_PDOR *((volatile unsigned*)(GPIOA_BASE + 0x0))    // GPIOA 데이터 출력 레지스터
#define GPIOA_PSOR *((volatile unsigned*)(GPIOA_BASE + 0x4))    // GPIOA 데이터 세트 출력 레지스터
#define GPIOA_PCOR *((volatile unsigned*)(GPIOA_BASE + 0x8))    // GPIOA 데이터 클리어 출력 레지스터
#define GPIOA_PTOR *((volatile unsigned*)(GPIOA_BASE + 0xC))    // GPIOA 데이터 토글 출력 레지스터
#define GPIOA_PDIR *((volatile unsigned*)(GPIOA_BASE + 0x10))   // GPIOA 데이터 입력 레지스터
#define GPIOA_PDDR *((volatile unsigned*)(GPIOA_BASE + 0x14))   // GPIOA 데이터 방향 레지스터
#define GPIOA_PIDR *((volatile unsigned*)(GPIOA_BASE + 0x18))   // GPIOA 데이터 입력 레지스터

#define GPIOB_PDOR *((volatile unsigned*)(GPIOB_BASE + 0x0))    // GPIOB 데이터 출력 레지스터
#define GPIOB_PSOR *((volatile unsigned*)(GPIOB_BASE + 0x4))    // GPIOB 데이터 세트 출력 레지스터
#define GPIOB_PCOR *((volatile unsigned*)(GPIOB_BASE + 0x8))    // GPIOB 데이터 클리어 출력 레지스터
#define GPIOB_PTOR *((volatile unsigned*)(GPIOB_BASE + 0xC))    // GPIOB 데이터 토글 출력 레지스터
#define GPIOB_PDIR *((volatile unsigned*)(GPIOB_BASE + 0x10))   // GPIOB 데이터 입력 레지스터
#define GPIOB_PDDR *((volatile unsigned*)(GPIOB_BASE + 0x14))   // GPIOB 데이터 방향 레지스터
#define GPIOB_PIDR *((volatile unsigned*)(GPIOB_BASE + 0x18))   // GPIOB 데이터 입력 레지스터

#define GPIOC_PDOR *((volatile unsigned*)(GPIOC_BASE + 0x0))    // GPIOC 데이터 출력 레지스터
#define GPIOC_PSOR *((volatile unsigned*)(GPIOC_BASE + 0x4))    // GPIOC 데이터 세트 출력 레지스터
#define GPIOC_PCOR *((volatile unsigned*)(GPIOC_BASE + 0x8))    // GPIOC 데이터 클리어 출력 레지스터
#define GPIOC_PTOR *((volatile unsigned*)(GPIOC_BASE + 0xC))    // GPIOC 데이터 토글 출력 레지스터
#define GPIOC_PDIR *((volatile unsigned*)(GPIOC_BASE + 0x10))   // GPIOC 데이터 입력 레지스터
#define GPIOC_PDDR *((volatile unsigned*)(GPIOC_BASE + 0x14))   // GPIOC 데이터 방향 레지스터
#define GPIOC_PIDR *((volatile unsigned*)(GPIOC_BASE + 0x18))   // GPIOC 데이터 입력 레지스터

#define GPIOD_PDOR *((volatile unsigned*)(GPIOD_BASE + 0x0))    // GPIOD 데이터 출력 레지스터
#define GPIOD_PSOR *((volatile unsigned*)(GPIOD_BASE + 0x4))    // GPIOD 데이터 세트 출력 레지스터
#define GPIOD_PCOR *((volatile unsigned*)(GPIOD_BASE + 0x8))    // GPIOD 데이터 클리어 출력 레지스터
#define GPIOD_PTOR *((volatile unsigned*)(GPIOD_BASE + 0xC))    // GPIOD 데이터 토글 출력 레지스터
#define GPIOD_PDIR *((volatile unsigned*)(GPIOD_BASE + 0x10))   // GPIOD 데이터 입력 레지스터
#define GPIOD_PDDR *((volatile unsigned*)(GPIOD_BASE + 0x14))   // GPIOD 데이터 방향 레지스터
#define GPIOD_PIDR *((volatile unsigned*)(GPIOD_BASE + 0x18))   // GPIOD 데이터 입력 레지스터

#define GPIOE_PDOR *((volatile unsigned*)(GPIOE_BASE + 0x0))    // GPIOE 데이터 출력 레지스터
#define GPIOE_PSOR *((volatile unsigned*)(GPIOE_BASE + 0x4))    // GPIOE 데이터 세트 출력 레지스터
#define GPIOE_PCOR *((volatile unsigned*)(GPIOE_BASE + 0x8))    // GPIOE 데이터 클리어 출력 레지스터
#define GPIOE_PTOR *((volatile unsigned*)(GPIOE_BASE + 0xC))    // GPIOE 데이터 토글 출력 레지스터
#define GPIOE_PDIR *((volatile unsigned*)(GPIOE_BASE + 0x10))   // GPIOE 데이터 입력 레지스터
#define GPIOE_PDDR *((volatile unsigned*)(GPIOE_BASE + 0x14))   // GPIOE 데이터 방향 레지스터
#define GPIOE_PIDR *((volatile unsigned*)(GPIOE_BASE + 0x18))   // GPIOE 데이터 입력 레지스터

// PCR register
#define	PORTA_BASE	(0x40049000)    // port a base
#define	PORTB_BASE	(0x4004A000)    // port b base
#define	PORTC_BASE	(0x4004B000)    // port c base
#define	PORTD_BASE	(0x4004C000)    // port d base
#define	PORTE_BASE	(0x4004D000)    // port e base

#define PORTD_PCR15 *((volatile unsigned*)(PORTD_BASE + 0X3C))  // LED Red
#define PORTD_PCR16 *((volatile unsigned*)(PORTD_BASE + 0X40))  // LED Green

#define	PORTC_PCR3	*((volatile unsigned*)(PORTC_BASE + 0x0C))  // Segment a
#define	PORTC_PCR5	*((volatile unsigned*)(PORTC_BASE + 0x14))  // Segment b

#define PORTC_PCR6  *((volatile unsigned*)(PORTC_BASE + 0x18))  // LPUART1_RX
#define PORTC_PCR7  *((volatile unsigned*)(PORTC_BASE + 0x1C))  // LPUART1_TX

#define	PORTB_PCR8	*((volatile unsigned*)(PORTB_BASE + 0x20))  // Segment c
#define	PORTB_PCR9	*((volatile unsigned*)(PORTB_BASE + 0x24))  // Segment d
#define	PORTB_PCR10	*((volatile unsigned*)(PORTB_BASE + 0x28))  // Segment e
#define	PORTB_PCR11	*((volatile unsigned*)(PORTB_BASE + 0x2C))  // Segment f
#define	PORTB_PCR12	*((volatile unsigned*)(PORTB_BASE + 0x30))  // Segment g

#define	PORTD_PCR5	*((volatile unsigned*)(PORTD_BASE + 0x14))  // Segment dp(dot point)
#define PORTD_PCR6  *((volatile unsigned*)(PORTD_BASE + 0x18))  // Seg_clock (colon :)

#define PORTD_PCR10 *((volatile unsigned*)(PORTD_BASE + 0x28))  // DC motor (Clockwise, +)
#define PORTD_PCR11 *((volatile unsigned*)(PORTD_BASE + 0x2C))  // DC motor (Counter-clockwise, -)

#define	PORTE_PCR1	*((volatile unsigned*)(PORTE_BASE + 0x04))  // Digit 1 (Com 1)
#define	PORTE_PCR5	*((volatile unsigned*)(PORTE_BASE + 0x14))  // Digit 2 (Com 2)
#define	PORTE_PCR3	*((volatile unsigned*)(PORTE_BASE + 0x0C))  // Digit 3 (Com 3)
#define	PORTE_PCR14	*((volatile unsigned*)(PORTE_BASE + 0x38))  // Digit 4 (Com 4)
#define	PORTE_PCR15	*((volatile unsigned*)(PORTE_BASE + 0x3C))  // Digit 5 (Com 5)
#define	PORTE_PCR16	*((volatile unsigned*)(PORTE_BASE + 0x40))  // Digit 6 (Com 6)

#define	PORTA_PCR12	*((volatile unsigned*)(PORTA_BASE + 0x30))  // Switch 1
#define	PORTA_PCR13	*((volatile unsigned*)(PORTA_BASE + 0x34))  // Switch 2

#define	PORTC_PCR12	*((volatile unsigned*)(PORTC_BASE + 0x30))  // Switch 3
#define	PORTC_PCR13	*((volatile unsigned*)(PORTC_BASE + 0x34))  // Switch 4

#define PORTA_PCR14 *((volatile unsigned*)(PORTA_BASE + 0x38))  // Switch 5 (모드 전환용)

#define PORTA_PCR11 *((volatile unsigned*)(PORTA_BASE + 0x2C))  // LCD_D0
#define PORTA_PCR17 *((volatile unsigned*)(PORTA_BASE + 0x44))  // LCD_D1

#define PORTD_PCR12 *((volatile unsigned*)(PORTD_BASE + 0x30))  // LCD_D2
#define PORTD_PCR3  *((volatile unsigned*)(PORTD_BASE + 0x0C))  // LCD_D3

#define PORTB_PCR14 *((volatile unsigned*)(PORTB_BASE + 0x38))  // LCD_D4
#define PORTB_PCR15 *((volatile unsigned*)(PORTB_BASE + 0x3C))  // LCD_D5
#define PORTB_PCR16 *((volatile unsigned*)(PORTB_BASE + 0x40))  // LCD_D6
#define PORTB_PCR17 *((volatile unsigned*)(PORTB_BASE + 0x44))  // LCD_D7

#define PORTC_PCR16 *((volatile unsigned*)(PORTC_BASE + 0x40))  // Register Select (RS)
#define PORTC_PCR17 *((volatile unsigned*)(PORTC_BASE + 0x44))  // Read/Write (RW)
#define PORTE_PCR9  *((volatile unsigned*)(PORTE_BASE + 0x24))  // Enable (E)
#define MUX_BITS	8

// pin number configuration
#define PTD15       15  // LED Red
#define PTD16       16  // LED Green

#define PTC3		3   // Segment a
#define PTC5		5   // Segment b

#define PTC6        6   // LPUART1_RX
#define PTC7        7   // LPUART1_TX

#define PTB8		8   // Segment c
#define PTB9		9   // Segment d
#define PTB10		10  // Segment e
#define PTB11		11  // Segment f
#define PTB12		12  // Segment g

#define PTD5		5   // Segment dp
#define PTD6        6   // Seg_clock (colon :)

#define PTD10       10  // DC Motor (Clockwise, +)
#define PTD11       11  // DC Motor (Counter-clockwise, -)

#define PTE1		1   // Digit 1
#define PTE5		5   // Digit 2
#define PTE3		3   // Digit 3
#define PTE14		14  // Digit 4
#define PTE15		15  // Digit 5
#define PTE16		16  // Digit 6

#define PTA12		12  // Switch 1
#define PTA13		13  // Switch 2
#define PTC12		12  // Switch 3
#define PTC13		13  // Switch 4
#define PTA14       14  // Switch 5 (모드 전환)

#define PTA11       11  // LCD_D0
#define PTA17       17  // LCD_D1

#define PTD12       12  // LCD_D2
#define PTD3        3   // LCD_D3

#define PTB14       14  // LCD_D4
#define PTB15       15  // LCD_D5
#define PTB16       16  // LCD_D6
#define PTB17       17  // LCD_D7

#define PTC16       16  // Register Select (RS)
#define PTC17       17  // Read/Write (RW)

#define PTE9        9   // Enable (E)

// LPIT register
#define LPIT_BASE	(0x40037000)    // LPIT 기본 주소
#define LPIT_MCR	*((volatile unsigned*)(LPIT_BASE + 0x8))    // LPIT 모듈 제어 레지스터
#define LPIT_MSR	*((volatile unsigned*)(LPIT_BASE + 0xC))    // LPIT 모듈 상태 레지스터
#define LPIT_MIER	*((volatile unsigned*)(LPIT_BASE + 0x10))   // LPIT 모듈 인터럽트 활성화 레지스터
#define LPIT_TVAL0	*((volatile unsigned*)(LPIT_BASE + 0x20))  // LPIT 타이머 값 레지스터 (0번)
#define LPIT_TCTRL0	*((volatile unsigned*)(LPIT_BASE + 0x28)) // LPIT 타이머 제어 레지스터 (0번)

#define M_CEN_BIT		0   // 모듈 활성화 비트
#define TIF0_BIT		0   // 타이머 인터럽트 플래그 비트 (0번)
#define TIE0_BIT		0   // 타이머 인터럽트 활성화 비트 (0번)
#define MODE_BITS		2   // 타이머 모드 비트 위치
#define T_EN_BIT		0   // 타이머 활성화 비트 위치

// nvic register
#define NVIC_ISER_BASE	(0xE000E100)    // NVIC 인터럽트 세트 활성화 레지스터 기본 주소
#define NVIC_ISER1		*((volatile unsigned*)(NVIC_ISER_BASE + 0x4))   // NVIC 인터럽트 세트 활성화 레지스터 1
#define NVIC_ISER3		*((volatile unsigned*)(NVIC_ISER_BASE + 0xC))   // NVIC 인터럽트 세트 활성화 레지스터 3

#define NVIC_ICPR_BASE	(0xE000E280)    // NVIC 인터럽트 클리어 펜딩 레지스터 기본 주소
#define NVIC_ICPR1		*((volatile unsigned*)(NVIC_ICPR_BASE + 0x4))   // NVIC 인터럽트 클리어 펜딩 레지스터 1
#define NVIC_ICPR3		*((volatile unsigned*)(NVIC_ICPR_BASE + 0xC))   // NVIC 인터럽트 클리어 펜딩 레지스터 3

#define NVIC_IPR_BASE	(0xE000E400)    // NVIC 인터럽트 우선순위 레지스터 기본 주소
#define NVIC_IPR12		*((volatile unsigned*)(NVIC_IPR_BASE + 0x30))   // Lpit0 Timer
#define NVIC_IPR61		*((volatile unsigned char*)(NVIC_IPR_BASE + 0x3D))  // Port A
#define NVIC_IPR63		*((volatile unsigned char*)(NVIC_IPR_BASE + 0x3F))  // Port C

// nvic 초기화 함수
void NVIC_init_IRQs(void)
{
	// 타이머 인터럽트 (lpit0, irq 48)
	NVIC_ISER1 |= (1 << (48 % 32)); //lpit0 인터럽트 활성화
	NVIC_ICPR1 |= (1 << (48 % 32)); //펜딩 플래그 클리어
	NVIC_IPR12 &= ~(255 << (48 % 4));
	NVIC_IPR12 |= (10 << (48 % 4));	//우선순위 10

	// 스위치 1, 2, 5 인터럽트 (porta, irq 59)
	NVIC_ISER1 |= (1 << (59 % 32)); //porta 인터럽트 활성화
	NVIC_ICPR1 |= (1 << (59 % 32)); //플래그 클리어
	NVIC_IPR61 = 15;  //우선순위 15

	// 스위치 3, 4 인터럽트 (portc, irq 61)
	NVIC_ISER1 |= (1 << (61 % 32)); //portc 인터럽트 설정
	NVIC_ICPR1 |= (1 << (61 % 32)); //플래그 클리어
	NVIC_IPR63 = 15;  // 우선순위 15
}

//sw interrupt
#define PCR_PE_BIT	1   // Pull Enable bit
#define PCR_PS_BIT	0   // Pull Select bit

#define ISF_BIT		24  // Interrupt Status Flag bit
#define IRQC_BITS	16  // Interrupt Configuration bits

// 클럭 초기화 함수들 (헤더파일 연동)
void SOSC_init_8MHz(void)
{
	SCG->SOSCDIV = SCG_SOSCDIV_SOSCDIV1(1) | SCG_SOSCDIV_SOSCDIV2(1);
	SCG->SOSCCFG = SCG_SOSCCFG_RANGE(2) | SCG_SOSCCFG_EREFS_MASK;

	while(SCG->SOSCCSR & SCG_SOSCCSR_LK_MASK);
	SCG->SOSCCSR = SCG_SOSCCSR_SOSCEN_MASK;

	while(!(SCG->SOSCCSR & SCG_SOSCCSR_SOSCVLD_MASK));
}
void SPLL_init_160MHz(void)
{
	while(SCG->SPLLCSR & SCG_SPLLCSR_LK_MASK);
	SCG -> SPLLCSR &= ~SCG_SPLLCSR_SPLLEN_MASK;

	SCG -> SPLLDIV |= SCG_SPLLDIV_SPLLDIV1(2) | SCG_SPLLDIV_SPLLDIV2(3);

	SCG -> SPLLCFG = SCG_SPLLCFG_MULT(24);

	while(SCG -> SPLLCSR & SCG_SPLLCSR_LK_MASK);
	SCG -> SPLLCSR |= SCG_SPLLCSR_SPLLEN_MASK;

	while(!(SCG -> SPLLCSR & SCG_SPLLCSR_SPLLVLD_MASK));
}
void NormalRUNmode_80MHz(void)
{
	SCG -> SIRCDIV = SCG_SIRCDIV_SIRCDIV1(1) | SCG_SIRCDIV_SIRCDIV2(1);

	SCG -> RCCR = SCG_RCCR_SCS(6) | SCG_RCCR_DIVCORE(0b01) | SCG_RCCR_DIVBUS(0b01) | SCG_RCCR_DIVSLOW(0b10);

	while (((SCG -> CSR & SCG_CSR_SCS_MASK) >> SCG_CSR_SCS_SHIFT) != 6) {}
}

//세그먼트 충돌 방지 지연 함수
void delay_1ms(int ms)
{
	for(int i=0;i<ms;i++)
	{
		for(int j=0;j<8000;j++) // 약 1ms 주기
		{
			__asm("nop");
		}
	}
}

// 전역 변수 모음

#define SIGNAL_RED_TIME     600 //6초
#define SIGNAL_YELLOW_TIME  300 //3초
#define SIGNAL_GREEN_TIME   900 //9초

//스톱워치, 시계, 모터 모드 확인, 택시 미터기 모드 전역 변수
typedef enum {
    MODE_IDLE = 0,
    MODE_TIMER = 1,
    MODE_CLOCK = 2,
    MODE_MOTOR_Plus = 3,
    MODE_MOTOR_Minus = 4,
    MODE_TAXI = 5
} DisplayMode;

//UART 메뉴 상태 관리 변수
typedef enum {
    UART_MENU_MAIN = 0,
    UART_MENU_MODE1 = 1,
    UART_MENU_MODE2 = 2,
    UART_MENU_MODE3 = 3,
    UART_MENU_MODE4 = 4,
    UART_MENU_MODE5 = 5
} UARTMenuState;

//택시 미터기 모드 관련 변수
typedef enum {
    TAXI_MODE_IDLE = 0, //빨간불, 정지
    TAXI_MODE_BRAKE = 1, //노란불, 감속
    TAXI_MODE_NORMAL = 2 //초록불, 주행
} TaxiMode;

//신호등 관련 변수 및 지속 시간 설정
typedef enum {
    SIGNAL_RED = 0,    //PTD15 ON, PTD16 OFF  (6초)
    SIGNAL_YELLOW = 1, //PTD15 ON, PTD16 ON   (2초)
    SIGNAL_GREEN = 2   //PTD15 OFF, PTD16 ON   (6초)
} SignalState;

static volatile SignalState signalState = SIGNAL_GREEN;
static volatile int signalTimer = 0; //초기값
static volatile int signalDuration = SIGNAL_GREEN_TIME; //6초, 10ms 단위

static volatile UARTMenuState uartMenuState = UART_MENU_MAIN; //시작 모드: 메인 화면

static volatile DisplayMode currentMode = MODE_IDLE; //시작 모드: Idle
static volatile int isTimerRunning = 1; //타이머 작동 상태 변수 (0: 정지, 1: 작동)

//택시 미터기 변수
//adc 값을 읽고 모터 표시값 계산해주는 변수
static volatile int motorDuty = 0; //기본 듀티값 0 (0 ~ 999 범위, FTM2_MOD 기준)
static volatile int CreepDuty = 300; //크립 현상을 구현하기 위한 듀티값 (0~999 범위)
static volatile int isMotorForward = 1; //1: 시계, 0: 반시계
static volatile int taxiMeterValue = 0; //현재 미터기 요금
static volatile int dutyAtYellow = 0; //노란불 진입 시의 motorDuty 저장

static volatile int taxiModeActive = 1; //MODE_TAXI에서만 신호등 자동 전환

//타이머 모드용 변수
static volatile int baseValue = 0;	//초기값
static volatile int timerCounter = 0;	//타이머 카운터
static volatile int Counter = 0;	//업데이트 주기 카운터

//시계 모드용 변수
static volatile int clockSeconds = 0; //초 (0~59)
static volatile int clockMinutes = 0; //분 (0~59)
static volatile int clockHours = 0;   //시 (0~23)

//모터 모드 표시 변수
typedef enum {
    MOTOR_UNIT_RPM = 0,
    MOTOR_UNIT_MPS = 1,
    MOTOR_UNIT_PERCENT = 2,
    MOTOR_UNIT_VOLT = 3
} MotorUnit;

//모터 모드용 변수
static volatile MotorUnit motorUnit = MOTOR_UNIT_RPM; //초기 모터 표시 단위 (RPM)
static volatile int adcValue = 0; // adc 값 저장 변수

//물리 상수 (임시, 보드 및 모터 사양에 따라 변경 필요)
#define MOTOR_MAX_RPM 84        //adc 최대일 때의 RPM
#define WHEEL_CIRC_M_X100 20       //바퀴 둘레(m) * 100 -> 0.2m => 20 (used to get m/s * 100)
#define SUPPLY_V_X100 1200        //공급전압*100 (12.00V -> 1200)

void PORT_init()
{
    PCC_PORTA |= (1 << CGC_BIT);
    PCC_PORTB |= (1 << CGC_BIT);
    PCC_PORTC |= (1 << CGC_BIT);
    PCC_PORTD |= (1 << CGC_BIT);
    PCC_PORTE |= (1 << CGC_BIT);

    PORTA_PCR12 &= ~((0b111)<<MUX_BITS); //sw 1
    PORTA_PCR12 |= ((0b001)<<MUX_BITS);
    PORTA_PCR12 |= (1<<PCR_PE_BIT); //스위치 활성화
    PORTA_PCR12 |= (1<<PCR_PS_BIT); //스위치 뗀 상황으로 초기화
    PORTA_PCR12 &= ~((0b1111) << IRQC_BITS); //IRQC 클리어
    PORTA_PCR12 |= ((0b1010) << IRQC_BITS); //IRQC = 1010 (falling edge)

    PORTA_PCR13 &= ~((0b111)<<MUX_BITS); //sw 2
    PORTA_PCR13 |= ((0b001)<<MUX_BITS);
    PORTA_PCR13 |= (1<<PCR_PE_BIT); //스위치 활성화
    PORTA_PCR13 |= (1<<PCR_PS_BIT); //스위치 뗀 상황으로 초기화
    PORTA_PCR13 &= ~((0b1111) << IRQC_BITS); //IRQC 클리어
    PORTA_PCR13 |= ((0b1010) << IRQC_BITS); //IRQC = 1010 (falling edge)

    PORTC_PCR12 &= ~((0b111)<<MUX_BITS); //sw 3
    PORTC_PCR12 |= ((0b001)<<MUX_BITS);
    PORTC_PCR12 |= (1<<PCR_PE_BIT); //스위치 활성화
    PORTC_PCR12 |= (1<<PCR_PS_BIT); //스위치 뗀 상황으로 초기화
    PORTC_PCR12 &= ~((0b1111) << IRQC_BITS); //IRQC 클리어
    PORTC_PCR12 |= ((0b1010) << IRQC_BITS); //IRQC = 1010 (falling edge)

    PORTC_PCR13 &= ~((0b111)<<MUX_BITS); //sw 4
    PORTC_PCR13 |= ((0b001)<<MUX_BITS);
    PORTC_PCR13 |= (1<<PCR_PE_BIT); //스위치 활성화
    PORTC_PCR13 |= (1<<PCR_PS_BIT); //스위치 뗀 상황으로 초기화
    PORTC_PCR13 &= ~((0b1111) << IRQC_BITS); //IRQC 클리어
    PORTC_PCR13 |= ((0b1010) << IRQC_BITS); //IRQC = 1010 (falling edge)

    PORTA_PCR14 &= ~((0b111)<<MUX_BITS); //sw 5
    PORTA_PCR14 |= ((0b001)<<MUX_BITS);
    PORTA_PCR14 |= (1<<PCR_PE_BIT); //스위치 활성화
    PORTA_PCR14 |= (1<<PCR_PS_BIT); //스위치 뗀 상황으로 초기화
    PORTA_PCR14 &= ~((0b1111) << IRQC_BITS); //IRQC 클리어
    PORTA_PCR14 |= ((0b1010) << IRQC_BITS); //IRQC = 1010 (falling edge)

    PORTE_PCR1 &= ~((0b111)<<MUX_BITS); //com 1
    PORTE_PCR1 |= ((0b001)<<MUX_BITS);
    PORTE_PCR5 &= ~((0b111)<<MUX_BITS); //com 2
    PORTE_PCR5 |= ((0b001)<<MUX_BITS);
    PORTE_PCR3 &= ~((0b111)<<MUX_BITS); //com 3
    PORTE_PCR3 |= ((0b001)<<MUX_BITS);
    PORTE_PCR14 &= ~((0b111)<<MUX_BITS); //com 4
    PORTE_PCR14 |= ((0b001)<<MUX_BITS);
    PORTE_PCR15 &= ~((0b111)<<MUX_BITS); //com 5
    PORTE_PCR15 |= ((0b001)<<MUX_BITS);
    PORTE_PCR16 &= ~((0b111)<<MUX_BITS); //com 6
    PORTE_PCR16 |= ((0b001)<<MUX_BITS);

    PORTC_PCR3 &= ~((0b111)<<MUX_BITS); //seg a
    PORTC_PCR3 |= ((0b001)<<MUX_BITS);
    PORTC_PCR5 &= ~((0b111)<<MUX_BITS); //seg b
    PORTC_PCR5 |= ((0b001)<<MUX_BITS);

    PORTB_PCR8 &= ~((0b111)<<MUX_BITS); //seg c
    PORTB_PCR8 |= ((0b001)<<MUX_BITS);
    PORTB_PCR9 &= ~((0b111)<<MUX_BITS); //seg d
    PORTB_PCR9 |= ((0b001)<<MUX_BITS);
    PORTB_PCR10 &= ~((0b111)<<MUX_BITS); //seg e
    PORTB_PCR10 |= ((0b001)<<MUX_BITS);
    PORTB_PCR11 &= ~((0b111)<<MUX_BITS); //seg f
    PORTB_PCR11 |= ((0b001)<<MUX_BITS);
    PORTB_PCR12 &= ~((0b111)<<MUX_BITS); //seg g
    PORTB_PCR12 |= ((0b001)<<MUX_BITS);

    PORTD_PCR5 &= ~((0b111)<<MUX_BITS); //seg dp
    PORTD_PCR5 |= ((0b001)<<MUX_BITS);
    PORTD_PCR6 &= ~((0b111)<<MUX_BITS); //seg_clock (colon :)
    PORTD_PCR6 |= ((0b001)<<MUX_BITS);

    PORTD_PCR15 &= ~((0b111) << MUX_BITS); //led red
    PORTD_PCR15 |= ((0b001) << MUX_BITS);
    PORTD_PCR16 &= ~((0b111) << MUX_BITS); //led green
    PORTD_PCR16 |= ((0b001) << MUX_BITS);

    //모터는 mux 010
    PORTD_PCR10 &= ~((0b111) << MUX_BITS); //clear bits
    PORTD_PCR10 |= ((0b010) << MUX_BITS);  //clockwise (+)
    PORTD_PCR11 &= ~((0b111) << MUX_BITS); //clear bits
    PORTD_PCR11 |= ((0b010) << MUX_BITS);  //counter-clockwise (-)

    //UART 또한 mux 010
    PORTC_PCR6 &= ~((0b111) << MUX_BITS); //LPUART1_RX (ALT2)
    PORTC_PCR6 |= ((0b010) << MUX_BITS);
    PORTC_PCR7 &= ~((0b111) << MUX_BITS); //LPUART1_TX (ALT2)
    PORTC_PCR7 |= ((0b010) << MUX_BITS);

    //lcd는 gpio를 이용하므로 mux 001
    PORTA_PCR11 &= ~((0b111) << MUX_BITS); //D0
    PORTA_PCR11 |= ((0b001) << MUX_BITS);
    PORTA_PCR17 &= ~((0b111) << MUX_BITS); //D1
    PORTA_PCR17 |= ((0b001) << MUX_BITS);

    PORTD_PCR12 &= ~((0b111) << MUX_BITS); //D2
    PORTD_PCR12 |= ((0b001) << MUX_BITS);
    PORTD_PCR3 &= ~((0b111) << MUX_BITS); //D3
    PORTD_PCR3 |= ((0b001) << MUX_BITS);

    PORTB_PCR14 &= ~((0b111) << MUX_BITS); //D4
    PORTB_PCR14 |= ((0b001) << MUX_BITS);
    PORTB_PCR15 &= ~((0b111) << MUX_BITS); //D5
    PORTB_PCR15 |= ((0b001) << MUX_BITS);
    PORTB_PCR16 &= ~((0b111) << MUX_BITS); //D6
    PORTB_PCR16 |= ((0b001) << MUX_BITS);
    PORTB_PCR17 &= ~((0b111) << MUX_BITS); //D7
    PORTB_PCR17 |= ((0b001) << MUX_BITS);


    PORTC_PCR16 &= ~((0b111) << MUX_BITS); //RS
    PORTC_PCR16 |= ((0b001) << MUX_BITS);
    PORTC_PCR17 &= ~((0b111) << MUX_BITS); //RW
    PORTC_PCR17 |= ((0b001) << MUX_BITS);

    PORTE_PCR9 &= ~((0b111) << MUX_BITS); //enable (E9 -> PTE9)
    PORTE_PCR9 |= ((0b001) << MUX_BITS);

    //모터는 pddr 설정할 필요 없음.
    //스위치는 입력(0), LED는 출력(1)으로 설정
    GPIOA_PDDR &= ~((1 << PTA12) | (1 << PTA13) | (1 << PTA14)); //sw1, 2, 5
    GPIOC_PDDR &= ~((1 << PTC12) | (1 << PTC13)); //sw3, 4

    GPIOA_PDDR |= (1 << PTA11) | (1 << PTA17);
    GPIOE_PDDR |= (1 << PTE9); //enable 출력 설정 (E9 -> PTE9)
    GPIOB_PDDR |=  (1 << PTB8) | (1 << PTB9) | (1 << PTB10) |
    			(1 << PTB11) | (1 << PTB12) | (1 << PTB14) |
                (1 << PTB15) | (1 << PTB16) | (1 << PTB17);
    GPIOC_PDDR |= (1 << PTC3) | (1 << PTC5) | (1 << PTC16) | (1 << PTC17);
    GPIOD_PDDR |= (1 << PTD5) | (1 << PTD6) | (1 << PTD15) |
                (1 << PTD16) | (1 << PTD12) | (1 << PTD3);
    GPIOE_PDDR |= (1 << PTE1) | (1 << PTE5) | (1 << PTE3) |
				  (1 << PTE14) | (1 << PTE15) | (1 << PTE16);
}

//신호등 제어 함수
void SetSignalLight(SignalState state)
{
    if(currentMode != MODE_TAXI) {
        GPIOD_PSOR |= (1 << PTD15); //red off
        GPIOD_PSOR |= (1 << PTD16); //green off
        return;
    }

    switch(state) {
        case SIGNAL_GREEN:
            GPIOD_PSOR |= (1 << PTD15); //red off
            GPIOD_PCOR |= (1 << PTD16); //green on
            signalDuration = SIGNAL_GREEN_TIME;
            break;
        case SIGNAL_YELLOW:
            GPIOD_PCOR |= (1 << PTD15); //red on
            GPIOD_PCOR |= (1 << PTD16); //green on
            signalDuration = SIGNAL_YELLOW_TIME;
            dutyAtYellow = motorDuty; //노란불 진입 시 현재 속도 저장
            break;

        case SIGNAL_RED:
            GPIOD_PCOR |= (1 << PTD15); //red on
            GPIOD_PSOR |= (1 << PTD16); //green off
            signalDuration = SIGNAL_RED_TIME;
            break;
    }
    signalState = state;
    signalTimer = 0;
}

//segment 숫자 config.
void set7segmentNumclear()
{
	GPIOB_PCOR |= (1 << PTB8)|(1 << PTB9)|(1 << PTB10)|(1 << PTB11)|(1 << PTB12);
	GPIOC_PCOR |= (1 << PTC3)|(1 << PTC5);
	GPIOD_PCOR |= (1 << PTD5);
}
void set7segmentNum0()
{
	GPIOB_PSOR |= (1 << PTB8)|(1 << PTB9)|(1 << PTB10)|(1 << PTB11);
	GPIOB_PCOR |= (1 << PTB12);
	GPIOC_PSOR |= (1 << PTC3)|(1 << PTC5);
	GPIOD_PCOR |= (1 << PTD5);
}
void set7segmentNum1()
{
	GPIOB_PSOR |= (1 << PTB8);
	GPIOB_PCOR |= (1 << PTB9)|(1 << PTB10)|(1 << PTB11)|(1 << PTB12);
	GPIOC_PSOR |= (1 << PTC5);
	GPIOC_PCOR |= (1 << PTC3);
	GPIOD_PCOR |= (1 << PTD5);
}
void set7segmentNum2()
{
	GPIOB_PSOR |= (1 << PTB9)|(1 << PTB10)|(1 << PTB12);
	GPIOB_PCOR |= (1 << PTB8)|(1 << PTB11);
	GPIOC_PSOR |= (1 << PTC3)|(1 << PTC5);
	GPIOD_PCOR |= (1 << PTD5);
}
void set7segmentNum3()
{
	GPIOB_PSOR |= (1 << PTB8)|(1 << PTB9)|(1 << PTB12);
	GPIOB_PCOR |= (1 << PTB10)|(1 << PTB11);
	GPIOC_PSOR |= (1 << PTC3)|(1 << PTC5);
	GPIOD_PCOR |= (1 << PTD5);
}
void set7segmentNum4()
{
	GPIOB_PSOR |= (1 << PTB8)|(1 << PTB11)|(1 << PTB12);
	GPIOB_PCOR |= (1 << PTB9)|(1 << PTB10);
	GPIOC_PSOR |= (1 << PTC5);
	GPIOC_PCOR |= (1 << PTC3);
	GPIOD_PCOR |= (1 << PTD5);
}
void set7segmentNum5()
{
	GPIOB_PSOR |= (1 << PTB8)|(1 << PTB9)|(1 << PTB11)|(1 << PTB12);
	GPIOB_PCOR |= (1 << PTB10);
	GPIOC_PSOR |= (1 << PTC3);
	GPIOC_PCOR |= (1 << PTC5);
	GPIOD_PCOR |= (1 << PTD5);
}
void set7segmentNum6()
{
	GPIOB_PSOR |= (1 << PTB8)|(1 << PTB9)|(1 << PTB10)|(1 << PTB11)|(1 << PTB12);
	GPIOC_PSOR |= (1 << PTC3);
	GPIOC_PCOR |= (1 << PTC5);
	GPIOD_PCOR |= (1 << PTD5);
}
void set7segmentNum7()
{
	GPIOB_PSOR |= (1 << PTB8)|(1 << PTB11);
	GPIOB_PCOR |= (1 << PTB9)|(1 << PTB10)|(1 << PTB12);
	GPIOC_PSOR |= (1 << PTC3)|(1 << PTC5);
	GPIOD_PCOR |= (1 << PTD5);
}
void set7segmentNum8()
{
	GPIOB_PSOR |= (1 << PTB8)|(1 << PTB9)|(1 << PTB10)|(1 << PTB11)|(1 << PTB12);
	GPIOC_PSOR |= (1 << PTC3)|(1 << PTC5);
	GPIOD_PCOR |= (1 << PTD5);
}
void set7segmentNum9()
{
	GPIOB_PSOR |= (1 << PTB8)|(1 << PTB9)|(1 << PTB11)|(1 << PTB12);
	GPIOB_PCOR |= (1 << PTB10);
	GPIOC_PSOR |= (1 << PTC3)|(1 << PTC5);
	GPIOD_PCOR |= (1 << PTD5);
}
void seg7segmentNumminus()
{
    GPIOB_PSOR |= (1 << PTB12);
    GPIOB_PCOR |= (1 << PTB8)|(1 << PTB9)|(1 << PTB10)|(1 << PTB11);
    GPIOC_PCOR |= (1 << PTC3)|(1 << PTC5);
    GPIOD_PCOR |= (1 << PTD5);
}

//숫자, 기호 출력 함수
void set7segmentNum(int num)
{
	switch(num)
	{
		case 0:
			set7segmentNum0(); //0
			break;
		case 1:
			set7segmentNum1(); //1
			break;
		case 2:
			set7segmentNum2(); //2
			break;
		case 3:
			set7segmentNum3(); //3
			break;
		case 4:
			set7segmentNum4(); //4
			break;
		case 5:
			set7segmentNum5(); //5
			break;
		case 6:
			set7segmentNum6(); //6
			break;
		case 7:
			set7segmentNum7(); //7
			break;
		case 8:
			set7segmentNum8(); //8
			break;
		case 9:
			set7segmentNum9(); //9
			break;
        case 10:
            seg7segmentNumminus(); //-
            break;
		default:
			set7segmentNumclear(); //초기화
	}
}

//digit 초기화 함수
void displayDigitClear()
{
	GPIOE_PCOR |= (1<<PTE1)|(1<<PTE5)|(1<<PTE3)|(1<<PTE14)|(1<<PTE15)|(1<<PTE16);
	set7segmentNumclear();
}

//디스플레이 함수
void displayDigit(int position, int num, int showDP)
{
	//먼저 모든 자릿수 끄기
	displayDigitClear();

	//숫자 패턴 설정
	set7segmentNum(num);

	//소수점 설정 (필요시)
	if(showDP) {
		GPIOD_PSOR |= (1 << PTD5);
	}

	//해당 자릿수 켜기
	switch(position) {
		case 1: GPIOE_PSOR |= (1<<PTE1); break;   //우측 1번째
		case 2: GPIOE_PSOR |= (1<<PTE5); break;   //우측 2번째
		case 3: GPIOE_PSOR |= (1<<PTE3); break;  //우측 3번째 (소수점 위치)
		case 4: GPIOE_PSOR |= (1<<PTE14); break;  //좌측 3번째
		case 5: GPIOE_PSOR |= (1<<PTE15); break;  //좌측 2번째
		case 6:                                 //좌측 1번째
            if(currentMode != MODE_MOTOR_Plus) {
                if(currentMode == MODE_MOTOR_Minus) {
                    if(motorUnit == MOTOR_UNIT_PERCENT || motorUnit == MOTOR_UNIT_VOLT) {
                    GPIOE_PCOR |= (1<<PTE16);
                } else {
                    seg7segmentNumminus();
                    GPIOE_PSOR |= (1<<PTE16);
                }
            } else {
                GPIOE_PSOR |= (1<<PTE16);
            }
        } break;
    }
}

//6자리 디스플레이 함수를 한번에 나타내주는 함수
void display6DigitNumber(int num)
{
	int digits[6]; //6자리로 표시

	//각 자릿수 추출 (우측부터)
	digits[0] = num % 10; num /= 10;        //0.01의 자리
	digits[1] = num % 10; num /= 10;        //0.1의 자리
	digits[2] = num % 10; num /= 10;        //1의 자리 (소수점 표시)
	digits[3] = num % 10; num /= 10;        //10의 자리
	digits[4] = num % 10; num /= 10;        //100의 자리
	digits[5] = num % 10;                   //1000의 자리

	//각 자릿수를 순차적으로 표시 (배열 방식으로 구현)
	for(int i = 0; i < 6; i++) {
		//2번째 자릿수(1의 자리)에만 소수점 표시
		int showDP;
		if(i == 2) {
			showDP = 1;  //소수점 켜기
		} else {
			showDP = 0;  //소수점 끄기
		}
		displayDigit(i+1, digits[i], showDP); //각각의 자리에 표시
		delay_1ms(2);  //2ms 지연 (충돌 방지)
	}
}

//초기값 표시 함수
void setBaseValue(int newBaseValue)
{
	baseValue = newBaseValue;
	timerCounter = 0;
	Counter = 0;
}
//현재 표시값을 계산해주는 변수 (기존 숫자 + 타이머 카운터)
int getDisplayValue(void)
{
	return baseValue + timerCounter;
}

//시계 지정 함수
void setClockTime(int hours, int minutes, int seconds)
{
    clockHours = hours % 24;   //0~23시
    clockMinutes = minutes % 60; //0~59분
    clockSeconds = seconds % 60; //0~59초
}
int getClockDisplayValue(void)
{
    return (clockHours * 10000) + (clockMinutes * 100) + clockSeconds;
}
//시계 모드를 위한 seg_clock (:) 온오프 함수 (clock 모드가 1이므로 state: 1에서 작동)
void setSegClock(int state)
{
    if(state) {
        GPIOD_PSOR |= (1 << PTD6); //켜기
    }
    else {
        GPIOD_PCOR |= (1 << PTD6); //끄기
    }
}

//모터 방향별 제어 함수
void setMotorDirectionPlus(void)
{
    //시계 방향
    GPIOD_PCOR |= (1 << PTD11);
    FTM2_C0V = 1000 - 1;
    FTM2_C1V = 0;
}

void setMotorDirectionMinus(void)
{
    //반시계 방향
    GPIOD_PSOR |= (1 << PTD11);
    FTM2_C0V = 0;
    FTM2_C1V = 1000 - 1;
}

//모터 제어용 펄스파 함수 생성 (ftm2 ch0 사용)
void FTM2_CH0_PWM(void)
{
    //ftm2 clocking
    PCC_FTM2 &= ~(1<<CGC_BIT); //disable clock to FTM2
    PCC_FTM2 &= ~((0b111) << PCS_BITS); //clear PCS bits
    PCC_FTM2 |= ((0b010) << PCS_BITS); //set PCS to SPLL (8MHz)
    PCC_FTM2 |= (1<<CGC_BIT); //enable clock to FTM2

    //ftm initialization
    FTM2_SC &= ~((0b111) << PS_BITS); //clear PS bits
    FTM2_MOD = 1000 - 1; //set mod for 20kHz pwm (4MHz / 2 / 1000 = 20000Hz)
    FTM2_CNTIN = 0; //set counter initial value to 0

    //ftm2 ch0 setup (시계 방향)
    FTM2_C0SC |= (1<<MSB_BIT); //set MSB for edge-aligned pwm
    FTM2_C0SC |= (1<<ELSB_BIT); //set ELSB for high-true pulses
    FTM2_C0SC &= ~(1<<ELSA_BIT); //clear ELSA for high-true pulses
    FTM2_C0V = 400 - 1; //duty 40%

    //ftm2 ch1 setup (반시계 방향)
    FTM2_C1SC |= (1<<MSB_BIT);
    FTM2_C1SC |= (1<<ELSB_BIT);
    FTM2_C1SC &= ~(1<<ELSA_BIT);
    FTM2_C1V = 0;  //초기: 0%

    //start FTM2 (ch0, ch1 모두 활성화)
    FTM2_SC |= ((0b001) << PS_BITS); //set PS to system clock (8MHz/2=4MHz)
    FTM2_SC |= ((0b01) << CLKS_BITS); //set CLKS to system clock
    FTM2_SC |= (1<<PWMEN0_BIT); //enable pwm on ch0
    FTM2_SC |= (1<<PWMEN1_BIT);
}

//adc0 지정 함수
void ADC0_init(void)
{
	//adc0 클럭 할당
	PCC_ADC0 &= ~(1<<CGC_BIT);
	PCC_ADC0 &= ~((0b111)<<PCS_BITS);
	PCC_ADC0 |= ((0b001)<<PCS_BITS);
	PCC_ADC0 |= (1<<CGC_BIT);

	//adc0 실행
	ADC0_SC1A |= ((0b111111)<<ADCH_BITS);

	ADC0_CFG1 &= ~((0b11)<<ADIV_BITS);
	ADC0_CFG1 &= ~((0b11)<<MODE_BITS);
	ADC0_CFG1 |= ((0b01)<<MODE_BITS);
	ADC0_CFG1 |= ((0b01)<<ADIV_BITS);

	ADC0_CFG2 &= ~(255<<SMPLTS_BITS);
	ADC0_CFG2 |= (12<<SMPLTS_BITS);

	ADC0_SC2 &= ~(1<<ADTRG_BIT);
    ADC0_SC3 = 0;
}

//adc 시작 함수
void adc_start(void)
{
	ADC0_SC1A &= ~((0b111111)<<ADCH_BITS); //adch bit 초기화
	ADC0_SC1A |= (ADC0_SE12<<ADCH_BITS); //컨버전 시작
}

//adc 결과값 읽는 함수
uint32_t read_adc_chx(void)
{
	while((ADC0_SC1A & (1<<COCO_BIT)) == 0) {}

	return ADC0_RA;
}

//adc 값을 pwm 듀티비로 변환하는 함수
void update_pwm_from_adc(int adcValue)
{
    int duty = (adcValue * (FTM2_MOD + 1)) / (4096 - 1); //12비트 = 4096 기준
    if (duty > FTM2_MOD) duty = FTM2_MOD;

    //모드 3
    if(currentMode == MODE_MOTOR_Plus) {
        FTM2_C0V = duty; //시계 방향
        FTM2_C1V = 0;
    } else if(currentMode == MODE_MOTOR_Minus){
        FTM2_C0V = 0;
        FTM2_C1V = duty; //반시계 방향
    }
}

//adc 값 대신 스위치를 이용해 pwm 듀티비를 변환해주는 함수
void UpdateMotorPWM(void)
{
    // motorDuty는 0~999 범위 (FTM2_MOD 기준)
    // FTM2_MOD = 999이므로 직접 사용 가능

    //빨간불일 경우 모터 정지
    if(signalState == SIGNAL_RED) {
        FTM2_C0V = 0;
        FTM2_C1V = 0;
    } else {
        //노란불, 초록불: motorDuty에 따라 PWM을 설정할 수 있음
        if(isMotorForward) {
            //시계 방향, 전진
            FTM2_C0V = motorDuty;
            FTM2_C1V = 0;
        } else {
            //반시계 방향, 후진
            FTM2_C0V = 0;
            FTM2_C1V = motorDuty;
        }
    }
}

//adc 값을 읽고 모터 표시값 계산해주는 변수
int getMotorDisplayValue(void)
{
    //duty 계산
    int duty = (adcValue * (FTM2_MOD + 1)) / (4096 - 1); //12비트 = 4096 기준
    //기존 duty 보관(percent, volt)
    int originalDuty = duty;
    //모터가 돌지 않는 영역 (50% 미만으로 설정)
    int deadzone = 500; //약 duty cycle 50% 이상부터 모터가 돌기 시작
    //RPM, cm/s에서 D = 50% 아래부분은 절삭하기 위한 조건문
    if(duty < deadzone) {
        duty = 0; //모터가 돌지 않는 영역
    } else {
        duty = (duty - deadzone) * 2; //데드존 보정 (_/ 모양 스케일링)
        if (duty > FTM2_MOD) duty = FTM2_MOD; //최대값 제한 (D = 100%, V = 12V를 넘어가는 걸 막기 위함)
    }
    //표시 모드에 따른 값 계산
    switch(motorUnit) {
        case MOTOR_UNIT_RPM: {
            //rpm = D * max rpm * 100 (xxx.00 rpm)
            int rpm = (duty * MOTOR_MAX_RPM) / (FTM2_MOD);
            return rpm * 100;
        }
        case MOTOR_UNIT_MPS: {
            //cm/s계산
            int rpm = (duty * MOTOR_MAX_RPM) / (FTM2_MOD);
            //m/s * 100 = rpm * 바퀴 둘레 / 60
            int mps_x100 = (rpm * WHEEL_CIRC_M_X100) / 60;
            return mps_x100 * 100; //cm 기준
        }
        case MOTOR_UNIT_PERCENT: {
            //percent * 100 (예: 75.00% -> 7500)
            int pct_x100 = (originalDuty * 10000) / (FTM2_MOD);
            if (pct_x100 > 10000) pct_x100 = 10000; //최대 100.00%
            return pct_x100;
        }
        case MOTOR_UNIT_VOLT: {
            //공급 전압 * 100 (예: 공급 전압이 12.00V -> 1200)
            int v_x100 = (originalDuty * SUPPLY_V_X100) / (FTM2_MOD);
            if (v_x100 > SUPPLY_V_X100) v_x100 = SUPPLY_V_X100; //최대 전압 제한 (12.00V)
            return v_x100;
        }
        default:
            return 0;
    }
}

//부동소수점 연산을 위한 누적 카운터 (모터 연산에 필요)
static int floatingcounter_sw3 = 0; //sw3 전용
static int floatingcounter_decay = 0; //자동 감속
static int floatingcounter_dual = 0; //양발 운전

//모터 듀티값 폴링으로 제어하는 함수
void UpdateMotorDuty(void)
{
    //스위치 상태 읽기
    int sw3_pressed;
    int sw4_pressed;
    unsigned int portc_status;

    // PORTC 인터럽트 플래그 미리 클리어 (폴링 중 인터럽트 간섭 방지)
    PORTC_PCR12 |= (1 << ISF_BIT);
    PORTC_PCR13 |= (1 << ISF_BIT);

    // GPIO 상태 읽기 (active low: 0=pressed, 1=released)
    portc_status = GPIOC_PDIR;

    // PTC12 (SW3) - 브레이크: 핀이 0(LOW)이면 눌린 상태
    if((portc_status & (1 << PTC12)) == 0) {
        sw3_pressed = 1;  // 눌림
    } else {
        sw3_pressed = 0;  // 안 눌림
    }

    // PTC13 (SW4) - 액셀: 핀이 0(LOW)이면 눌린 상태
    if((portc_status & (1 << PTC13)) == 0) {
        sw4_pressed = 1;  // 눌림
    } else {
        sw4_pressed = 0;  // 안 눌림
    }

    if(sw3_pressed && sw4_pressed) {
        //양발 운전하는 정신나간 사람 (duty: -50/초)
        //10틱마다 (100ms) -50
        floatingcounter_dual++;
        if(floatingcounter_dual >= 2) {
            motorDuty -= 1;
            floatingcounter_dual = 0;
        }
        floatingcounter_decay = 0;
    } else if(sw3_pressed) {
        //브레이크, 급정지 (duty: -250/초)
        floatingcounter_sw3++;
        if(floatingcounter_sw3 >= 2) {
            motorDuty -= 5; //2ms당 5 감소
            floatingcounter_sw3 = 0;
        }
        floatingcounter_decay = 0;
        floatingcounter_dual = 0;
    } else if(sw4_pressed) {
        //액셀, 가속 (duty: +200/초)
        motorDuty += 2; //10ms당 20 증가
        floatingcounter_sw3 = 0;
        floatingcounter_decay = 0;
        floatingcounter_dual = 0;
    } else {
        //발 다 뗀 상태 (브레이크 미사용)
        // motorDuty는 600 이상으로 유지 (액셀 해제 후 자동 유지속도)
        floatingcounter_sw3 = 0;
        floatingcounter_decay = 0;
        floatingcounter_dual = 0;
    }

    //범위 제한 (브레이크 누르지 않으면 600 이상으로 유지)
    int min_duty = 600; // 최저 속도

    if(motorDuty < min_duty && !sw3_pressed) {
        motorDuty = min_duty; // 브레이크 미사용 시 600으로 고정
    }

    if(motorDuty < 0) {
        motorDuty = 0;
    }
    if(motorDuty > 999) {
        motorDuty = 999;
    }

    UpdateMotorPWM();
}

//UART 데이터를 LCD로 연결된 포트에 전달해주는 함수 모음집
//LCD 지정한 번지에 문자열 쓰라는 신호 매핑해주는 함수
void LCD_Write_8bit(unsigned char data, unsigned char rs)
{
    //rs(register select) 설정 (0: 커맨드, 1: 데이터)
    if(rs) {
        GPIOC_PSOR |= (1<<PTC16); //RS = 1, 데이터 모드
    } else {
        GPIOC_PCOR |= (1<<PTC16); //RS = 0, 커맨드 모드
    }

    //rw(read/write)는 항상 0
    GPIOC_PCOR |= (1<<PTC17);

    //데이터 비트 설정
    if(data & 0x01) GPIOA_PSOR |= (1 << PTA11);
        else GPIOA_PCOR |= (1 << PTA11); //D0
    if(data & 0x02) GPIOA_PSOR |= (1 << PTA17);
        else GPIOA_PCOR |= (1 << PTA17); //D1
    if(data & 0x04) GPIOD_PSOR |= (1 << PTD12);
        else GPIOD_PCOR |= (1 << PTD12); //D2
    if(data & 0x08) GPIOD_PSOR |= (1 << PTD3);
        else GPIOD_PCOR |= (1 << PTD3);  //D3
    if(data & 0x10) GPIOB_PSOR |= (1 << PTB14);
        else GPIOB_PCOR |= (1 << PTB14); //D4
    if(data & 0x20) GPIOB_PSOR |= (1 << PTB15);
        else GPIOB_PCOR |= (1 << PTB15); //D5
    if(data & 0x40) GPIOB_PSOR |= (1 << PTB16);
        else GPIOB_PCOR |= (1 << PTB16); //D6
    if(data & 0x80) GPIOB_PSOR |= (1 << PTB17);
        else GPIOB_PCOR |= (1 << PTB17); //D7

    delay_1ms(1); //데이터 안정화 대기

    //Enable 신호(High->Low) - 펄스 폭 확대 (E9 -> PTE9)
    GPIOE_PSOR |= (1<<PTE9); //high, 신호 1
    delay_1ms(2); //Enable 신호 지속 시간 증가
    GPIOE_PCOR |= (1<<PTE9); //low, 신호 0
    delay_1ms(2); //Enable 신호 후 안정화 시간
}

//LCD 커맨드 입력
void LCD_Write_Command(unsigned char cmd)
{
    LCD_Write_8bit(cmd, 0); //RS = 0 (커맨드 모드)
}

//ASCII 코드를 데이터로써 LCD에 전송
void LCD_Write_Data(unsigned char data)
{
    LCD_Write_8bit(data, 1); //RS = 1 (데이터 모드)
}

//LCD 초기화 함수(8비트)
void LCD_init(void)
{
    delay_1ms(100); //전원 안정화 대기

    LCD_Write_Command(0x38); //Function 설정, 8비트 2줄, 5x8 폰트 기준
    delay_1ms(20);

    LCD_Write_Command(0x38); //반복
    delay_1ms(20);

    LCD_Write_Command(0x38); //반복
    delay_1ms(20);

    //디스플레이 ON/OFF 제어
    LCD_Write_Command(0x0C); //Display On, Cursor Off, Blink Off
    delay_1ms(10);

    //모드 설정
    LCD_Write_Command(0x06); //커서 증가, 화면 이동 없음
    delay_1ms(10);

    //디스플레이 클리어
    LCD_Write_Command(0x01); //화면 클리어
    delay_1ms(10);
}

//LCD 커서 위치 설정
void LCD_Set_Cursor(unsigned char row, unsigned char col)
{
    unsigned char addr; //위치 설정 변수(address의 약어로 설정)

    if(row == 0) {
        addr = (0x80) + col; //첫줄: 0x80 ~ 0x8F 까지 입력
    } else {
        addr = (0xC0) + col; //두번째 줄: 0xC0 ~ 0xCF 까지 입력
    }

    LCD_Write_Command(addr);
    delay_1ms(1);
}

//문자를 ASCII 코드로 변환
void LCD_Write_Char(char c)
{
    LCD_Write_Data((unsigned char)c); //0~255 범위의 ASCII 코드로 전송
}

//LCD에 문자열을 ASCII 코드로 변환해서 전송해주는 함수 (문자열 string의 약어 str로 변수 설정)
void LCD_Print(const char *str)
{
    while(*str) {
        LCD_Write_Char(*str++);
        delay_1ms(1);
    }
}

//LCD 화면 클리어
void LCD_Clear(void)
{
    LCD_Write_Command(0x01);
    delay_1ms(2);
}

//LCD 윗줄에 쓸 1줄 설정 (자동으로 lcd에 반영)
void Set_LCD_Line1(const char *text)
{
    LCD_Set_Cursor(0, 0);  //첫 줄, 첫 열로 커서 이동

    int i = 0;
    //최대 16글자 또는 NULL 문자 도달할 때까지
    while(i < 16 && text[i] != '\0') {
        LCD_Write_Char(text[i]);  //각 문자를 ASCII 코드로 전송
        i++;
        delay_1ms(1);
    }
        //나머지 공간을 스페이스로 채우기
    while(i < 16) {
        LCD_Write_Char(' ');
        i++;
        delay_1ms(1);
    }
}

//LPUART1 통신 함수 모음집
//uart 초기화
//9600 baud, 1 stop bit, 8bit format, no parity
void LPUART1_init(void)
{
    //enable clock for LPUART1
    PCC_LPUART1 &= ~(1 << CGC_BIT); //disable clock to configure
    PCC_LPUART1 &= ~((0b111) << PCS_BITS); //clear PCS bits
    PCC_LPUART1 |= ((0b010) << PCS_BITS); //set PCS to SPLL (80MHz /2 = 40MHz)
    PCC_LPUART1 |= (1 << CGC_BIT); //enable clock

    //initialize  for 9600 baud
    LPUART1_BAUD &= ~(1<<M10_BIT); //clear M10 for 8bit mode
    LPUART1_BAUD &= ~(1<<SBNS_BIT); //clear SBNS for 1 stop bit

    LPUART1_BAUD &= ~((0b11111) << OSR_BITS); //clear OSR bits
    LPUART1_BAUD |= (15 << OSR_BITS); //set OSR to 15
    LPUART1_BAUD &= ~(8191 << SBR_BITS); //clear SBR bits (52, 0x34)
    LPUART1_BAUD |= (52 << SBR_BITS); //set SBR to 52 for 9600 baud

    //enable LPUART1 transmitter and receiver
    LPUART1_CTRL &= ~(1<<M7_BIT); //clear M7 for 8bit mode
    LPUART1_CTRL &= ~(1<<M_BIT); //clear M for 8bit mode
    LPUART1_CTRL &= ~(1<<PE_BIT); //clear PE for no parity

    LPUART1_CTRL |= (1<<TE_BIT); //enable transmitter
    LPUART1_CTRL |= (1<<RE_BIT); //enable receiver
}

//LPUART1 문자 전송 함수
void LPUART1_transmit_char(char send)
{
    //wait until transmit data register empty
    while((LPUART1_STAT & (1 << TDRE_BIT)) == 0);
    //send character
    LPUART1_DATA = send;
}

//LPUART1 문자열 전송 함수 (하나씩 전송함)
void LPUART1_transmit_string(char data_string[])
{
    uint32_t i = 0;
    //send chars one at a time
    while(data_string[i] != '\0') {
        LPUART1_transmit_char(data_string[i]);
        i++;
    }
}

//LPUART1 문자 수신 함수
char LPUART1_receive_char(void)
{
    char receive;
    while((LPUART1_STAT & (1 << RDRF_BIT)) == 0); //wait until receive data register full
    receive = LPUART1_DATA; //read received data
    return receive;
}

//UART 콘솔창 표시 함수
void display_main_menu(void)
{
    LPUART1_transmit_string("Mode: Idle\n\r");
    LPUART1_transmit_string("Select Mode\n\r");

    Set_LCD_Line1("Mode:Idle");

    LPUART1_transmit_string("0: Main Menu\n\r");
    LPUART1_transmit_string("1: Stopwatch\n\r");
    LPUART1_transmit_string("2: Clock\n\r");
    LPUART1_transmit_string("3: Motor+ (Clockwise)\n\r");
    LPUART1_transmit_string("4: Motor- (Counter-Clockwise)\n\r");
    LPUART1_transmit_string("5: Taxi Meter\n\r");
    LPUART1_transmit_string("> ");
}
//모드 1: 스톱워치
void display_mode1_menu(void)
{
    LPUART1_transmit_string("\n\rMODE 1: STOPWATCH\n\r");

    if(isTimerRunning) {
        LPUART1_transmit_string("Status: Resume\n\r");
        Set_LCD_Line1("Stopwatch:Run");
    } else {
        LPUART1_transmit_string("Status: Stop\n\r");
        Set_LCD_Line1("Stopwatch:Stop");
    }
    LPUART1_transmit_string("1: Set 0000.00\n\r");
    LPUART1_transmit_string("2: Set 1500.00\n\r");
    LPUART1_transmit_string("3: Set 3000.00\n\r");
    LPUART1_transmit_string("4: Toggle Stop/Resume\n\r");
    LPUART1_transmit_string("0: Back to Main Menu\n\r");
    LPUART1_transmit_string("> ");
}
//모드 2: 시계
void display_mode2_menu(void)
{
    LPUART1_transmit_string("\n\rMODE 2: CLOCK\n\r");

    if(isTimerRunning) {
        LPUART1_transmit_string("Status: Resume\n\r");
        Set_LCD_Line1("Clock:Run");
    } else {
        LPUART1_transmit_string("Status: Stop\n\r");
        Set_LCD_Line1("Clock:Stop");
    }
    LPUART1_transmit_string("1: +1 Second\n\r");
    LPUART1_transmit_string("2: +1 Minute\n\r");
    LPUART1_transmit_string("3: +1 Hour\n\r");
    LPUART1_transmit_string("4: Toggle Stop/Resume\n\r");
    LPUART1_transmit_string("0: Back to Main Menu\n\r");
    LPUART1_transmit_string("> ");
}
//모드 3: DC모터 +
void display_mode3_menu(void)
{
    LPUART1_transmit_string("\n\rMODE 3: MOTOR+\n\r");

    LPUART1_transmit_string("Unit: ");
    switch(motorUnit) {
        case MOTOR_UNIT_RPM: LPUART1_transmit_string("RPM\n\r"); break;
        case MOTOR_UNIT_MPS: LPUART1_transmit_string("cm/s\n\r"); break;
        case MOTOR_UNIT_PERCENT: LPUART1_transmit_string("Duty Cycle\n\r"); break;
        case MOTOR_UNIT_VOLT: LPUART1_transmit_string("Input Voltage\n\r"); break;
    }
    //측정 unit별 LCD 표시값은 한 번에 담기엔 너무 보기 번잡해서 따로 구현
    switch (motorUnit) {
        case MOTOR_UNIT_RPM: Set_LCD_Line1("Motor+Unit:RPM"); break;
        case MOTOR_UNIT_MPS: Set_LCD_Line1("Motor+Unit:cm/s"); break;
        case MOTOR_UNIT_PERCENT: Set_LCD_Line1("Motor+Unit:Duty"); break;
        case MOTOR_UNIT_VOLT: Set_LCD_Line1("Motor+Unit:Vin"); break;
    }

    LPUART1_transmit_string("1: RPM\n\r");
    LPUART1_transmit_string("2: cm/s\n\r");
    LPUART1_transmit_string("3: Duty Cycle\n\r");
    LPUART1_transmit_string("4: Input Volt\n\r");
    LPUART1_transmit_string("0: Back to Main Menu\n\r");
    LPUART1_transmit_string("> ");
}
//모드 4: DC모터 -
void display_mode4_menu(void)
{
    LPUART1_transmit_string("\n\rMODE 4: MOTOR-\n\r");

    LPUART1_transmit_string("Unit: ");
    switch(motorUnit) {
        case MOTOR_UNIT_RPM: LPUART1_transmit_string("RPM\n\r"); break;
        case MOTOR_UNIT_MPS: LPUART1_transmit_string("cm/s\n\r"); break;
        case MOTOR_UNIT_PERCENT: LPUART1_transmit_string("Duty Cycle\n\r"); break;
        case MOTOR_UNIT_VOLT: LPUART1_transmit_string("Input Voltage\n\r"); break;
    }
    //측정 unit별 LCD 표시값은 한 번에 담기엔 너무 보기 번잡해서 따로 구현
    switch (motorUnit) {
        case MOTOR_UNIT_RPM: Set_LCD_Line1("Motor-Unit:RPM"); break;
        case MOTOR_UNIT_MPS: Set_LCD_Line1("Motor-Unit:cm/s"); break;
        case MOTOR_UNIT_PERCENT: Set_LCD_Line1("Motor-Unit:Duty"); break;
        case MOTOR_UNIT_VOLT: Set_LCD_Line1("Motor-Unit:Vin"); break;
    }

    LPUART1_transmit_string("1: RPM\n\r");
    LPUART1_transmit_string("2: cm/s\n\r");
    LPUART1_transmit_string("3: Duty Cycle\n\r");
    LPUART1_transmit_string("4: Input Volt\n\r");
    LPUART1_transmit_string("0: Back to Main Menu\n\r");
    LPUART1_transmit_string("> ");
}

//신호등 상태 변경 시 LCD에 반영
void UpdateTaxiLCD(void)
{
    if(currentMode == MODE_TAXI) {
        if(isMotorForward) {
            switch(signalState) {
                case SIGNAL_RED:
                    Set_LCD_Line1("Red,Forward");
                    break;
                case SIGNAL_YELLOW:
                    Set_LCD_Line1("Yellow,Forward");
                    break;
                case SIGNAL_GREEN:
                    Set_LCD_Line1("Green,Forward");
                    break;
            }
        } else {
            switch(signalState) {
                case SIGNAL_RED:
                    Set_LCD_Line1("Red,Backward");
                    break;
                case SIGNAL_YELLOW:
                    Set_LCD_Line1("Yellow,Backward");
                    break;
                case SIGNAL_GREEN:
                    Set_LCD_Line1("Green,Backward");
                    break;
            }
        }
    }
}

//모드 5: 택시 미터기
void display_mode5_menu(void)
{
    LPUART1_transmit_string("\n\rMODE 5: TAXI METER\n\r");
    LPUART1_transmit_string("Signal: ");
    switch(signalState) {
        case SIGNAL_RED:
            LPUART1_transmit_string("RED\n\r");
            break;
        case SIGNAL_YELLOW:
            LPUART1_transmit_string("YELLOW\n\r");
            break;
        case SIGNAL_GREEN:
            LPUART1_transmit_string("GREEN\n\r");
            break;
    }

    LPUART1_transmit_string("Direction: ");
    if(isMotorForward) {
        LPUART1_transmit_string("Forward\n\r");
    } else {
        LPUART1_transmit_string("Backward\n\r");
    }
    LPUART1_transmit_string("\n\r1: Initialize Meter Fee\n\r");
    LPUART1_transmit_string("2: Toggle Direction\n\r");
    LPUART1_transmit_string("3: Brake\n\r");
    LPUART1_transmit_string("4: Accelerate\n\r");
    LPUART1_transmit_string("0: Back to Main Menu\n\r");
    LPUART1_transmit_string("> ");
}

//UART 모드 작동 처리 함수
void handle_uart_menu(void)
{
    //입력값을 받고 콘솔창으로 반환해주는 함수
    char input = LPUART1_receive_char();
    LPUART1_transmit_char(input);
    LPUART1_transmit_char('\n');
    LPUART1_transmit_char('\r');

    //각 모드별 경우에서 입력(input)을 받음
    //현재 모드 -> uart 상태 -> 각 모드별 출력 문구 순으로 진행
    switch(uartMenuState) {
        //========================================
        // UART 메인 메뉴
        //========================================
        case UART_MENU_MAIN:
            switch(input) {
                case '0':  //IDLE 모드
                    currentMode = MODE_IDLE;
                    uartMenuState = UART_MENU_MAIN;
                    display_main_menu();
                    break;

                case '1':  //스톱워치 모드
                    currentMode = MODE_TIMER;
                    uartMenuState = UART_MENU_MODE1;
                    display_mode1_menu();
                    break;

                case '2':  //시계 모드
                    currentMode = MODE_CLOCK;
                    uartMenuState = UART_MENU_MODE2;
                    display_mode2_menu();
                    break;

                case '3':  //모터+ (시계방향) 모드
                    currentMode = MODE_MOTOR_Plus;
                    uartMenuState = UART_MENU_MODE3;
                    display_mode3_menu();
                    break;

                case '4':  //모터- (반시계방향) 모드
                    currentMode = MODE_MOTOR_Minus;
                    uartMenuState = UART_MENU_MODE4;
                    display_mode4_menu();
                    break;

                case '5':  //택시 미터기 모드
                    currentMode = MODE_TAXI;
                    uartMenuState = UART_MENU_MODE5;
                    taxiModeActive = 1; //신호등 자동 전환 시작
                    SetSignalLight(SIGNAL_GREEN); //기본값: 초록불
                    UpdateTaxiLCD(); //택시 모드를 위한 lcd를 만들었으므로 이걸 이용
                    display_mode5_menu();
                    break;
                default:
                    LPUART1_transmit_string("Invalid input. Try again.\n\r");
                    display_main_menu();
                    break;
            }
            break;

        //========================================
        // MODE 1: 스톱워치
        //========================================
        case UART_MENU_MODE1:
            switch(input) {
                case '0':  //메인 메뉴 복귀
                    currentMode = MODE_IDLE;
                    uartMenuState = UART_MENU_MAIN;
                    display_main_menu();
                    break;

                case '1':  //0000.00으로 설정
                    setBaseValue(0);
                    display_mode1_menu();
                    break;

                case '2':  //1500.00으로 설정
                    setBaseValue(150000);
                    display_mode1_menu();
                    break;

                case '3':  //3000.00으로 설정
                    setBaseValue(300000);
                    display_mode1_menu();
                    break;

                case '4':  //스톱워치 시작/정지 토글
                    isTimerRunning = !isTimerRunning;
                    display_mode1_menu();
                    break;
                default:
                    LPUART1_transmit_string("Invalid input. Try again.\n\r");
                    display_mode1_menu();
                    break;
            }
            break;

        //========================================
        // MODE 2: 시계
        //========================================
        case UART_MENU_MODE2:
            switch(input) {
                case '0':  //메인 메뉴 복귀
                    currentMode = MODE_IDLE;
                    uartMenuState = UART_MENU_MAIN;
                    display_main_menu();
                    break;

                case '1':  //+1초
                    clockSeconds++;
                    if(clockSeconds >= 60) {
                        clockSeconds = 0;
                        clockMinutes++;
                    }
                    display_mode2_menu();
                    break;

                case '2':  //+1분
                    clockMinutes++;
                    if(clockMinutes >= 60) {
                        clockMinutes = 0;
                        clockHours++;
                    }
                    display_mode2_menu();
                    break;

                case '3':  //+1시간
                    clockHours++;
                    if(clockHours >= 24) clockHours = 0;
                    display_mode2_menu();
                    break;

                case '4':  //시계 시작/정지 토글
                    isTimerRunning = !isTimerRunning;
                    display_mode2_menu();
                    break;
                default:
                    LPUART1_transmit_string("Invalid input. Try again.\n\r");
                    display_mode2_menu();
                    break;
            }
            break;

        //========================================
        // MODE 3: 모터+ (시계방향)
        //========================================
        case UART_MENU_MODE3:
            switch(input) {
                case '0':  //메인 메뉴 복귀
                    currentMode = MODE_IDLE;
                    uartMenuState = UART_MENU_MAIN;
                    display_main_menu();
                    break;

                case '1':  //RPM 모드
                    motorUnit = MOTOR_UNIT_RPM;
                    display_mode3_menu(); break;

                case '2':  //cm/s 모드
                    motorUnit = MOTOR_UNIT_MPS;
                    display_mode3_menu(); break;

                case '3':  //Duty Cycle 모드
                    motorUnit = MOTOR_UNIT_PERCENT;
                    display_mode3_menu(); break;

                case '4':  //Input Voltage 모드
                    motorUnit = MOTOR_UNIT_VOLT;
                    display_mode3_menu(); break;
                default:
                    LPUART1_transmit_string("Invalid input. Try again.\n\r");
                    display_mode3_menu();
                    break;
            }
            break;

        //========================================
        // MODE 4: 모터- (반시계방향)
        //========================================
        case UART_MENU_MODE4:
            switch(input) {
                case '0':  //메인 메뉴 복귀
                    currentMode = MODE_IDLE;
                    uartMenuState = UART_MENU_MAIN;
                    display_main_menu();
                    break;

                case '1':  //RPM 모드
                    motorUnit = MOTOR_UNIT_RPM;
                    display_mode4_menu();
                    break;

                case '2':  //cm/s 모드
                    motorUnit = MOTOR_UNIT_MPS;
                    display_mode4_menu();
                    break;

                case '3':  //Duty Cycle 모드
                    motorUnit = MOTOR_UNIT_PERCENT;
                    display_mode4_menu();
                    break;

                case '4':  //Input Voltage 모드
                    motorUnit = MOTOR_UNIT_VOLT;
                    display_mode4_menu();
                    break;
                default:
                    LPUART1_transmit_string("Invalid input. Try again.\n\r");
                    display_mode4_menu();
                    break;
            }
            break;

        //========================================
        // MODE 5: 택시 미터기
        //========================================
        case UART_MENU_MODE5:
            switch(input) {
                case '0':  //메인 메뉴 복귀
                    currentMode = MODE_IDLE;
                    uartMenuState = UART_MENU_MAIN;
                    taxiModeActive = 0;  //신호등 자동 전환 종료
                    display_main_menu();
                    break;

                case '1':  //미터기 초기화 (0000.00)
                    taxiMeterValue = 0;
                    UpdateTaxiLCD();
                    display_mode5_menu();
                    break;

                case '2':  //전후진 토글  //전후진 토글
                    isMotorForward = !isMotorForward;
                    UpdateMotorPWM();
                    UpdateTaxiLCD();
                    display_mode5_menu();
                    break;

                case '3':  //브레이크 (감속)
                    motorDuty -= 100;
                    if(motorDuty < 100) {
                        motorDuty = 100;
                    }
                    display_mode5_menu();
                    break;

                case '4':  //액셀 (가속)
                    //액셀 (가속): 100씩 증가, 최대값 1000
                    motorDuty += 100;
                    if(motorDuty > 1000) {
                        motorDuty = 1000;
                    }
                    display_mode5_menu();
                    break;
                default:
                    LPUART1_transmit_string("Invalid input. Try again.\n\r");
                    display_mode5_menu();
                    break;
            }
            break;
    }
}

//메인 함수
int main(void)
{
    PORT_init();
    SOSC_init_8MHz();
    SPLL_init_160MHz();
    NormalRUNmode_80MHz();

    LCD_init(); //LCD
    delay_1ms(50); //초기화 후 충분한 대기

    NVIC_init_IRQs();
    LPIT0_init();
    ADC0_init();
    FTM2_CH0_PWM();
    LPUART1_init();

	delay_1ms(10); //0.01초 대기 (충돌 방지)

    //타이머, 시계 모드 초기값 설정
	setBaseValue(0);	//타이머 모드 초기 표시값 (xxxx.xx) 설정
    setClockTime(12, 0, 0); //시계 모드 초기 시간 설정 (12:00:00)

    setSegClock(0); //타이머 모드: 초기 seg_clock 끄기

    taxiMeterValue = 0; //0원부터 시작
    motorDuty = 600; //초기값 600 (60% 크리핑 상태)
    isMotorForward = 1; //전진 상태
    SetSignalLight(SIGNAL_GREEN); //초록불로 시작

    LPUART1_transmit_string("\n\r ==== Welcome to Multi Mode Interface!!! ==== \n\r");
    display_main_menu();

	for(;;)
	{
        int displayValue; //현재 표시할 값

        //UART 사용을 위한 메뉴 확인 (폴링 방식으로 구현)
        if((LPUART1_STAT & (1 << RDRF_BIT)) != 0) {
            handle_uart_menu(); //UART 통신 시 UART 입력 처리
        }

        //모드에 따른 표시값 설정
        if(currentMode == MODE_IDLE) {
            GPIOD_PSOR |= (1 << PTD15); //red off
            GPIOD_PSOR |= (1 << PTD16); //green off
            //idle 상태에서는 세그먼트 표시 안함.
        }
        else if(currentMode == MODE_TIMER) {
            GPIOD_PSOR |= (1 << PTD15); //red off
            GPIOD_PSOR |= (1 << PTD16); //green off
            displayValue = getDisplayValue(); //타이머 모드 표시값
            setSegClock(0); //타이머 모드: seg_clock (:) 끄기
        }
        else if(currentMode == MODE_CLOCK) {
            GPIOD_PSOR |= (1 << PTD15); //red off
            GPIOD_PSOR |= (1 << PTD16); //green off
            displayValue = getClockDisplayValue(); //시계 모드 표시값
            setSegClock(1); //시계 모드: seg_clock (:) 켜기
        }
        else if(currentMode == MODE_MOTOR_Plus) {
            GPIOD_PSOR |= (1 << PTD15); //red off
            GPIOD_PSOR |= (1 << PTD16); //green off
            //모터 모드 (+)
            setMotorDirectionPlus();
            adc_start();
            adcValue = read_adc_chx();
            update_pwm_from_adc(adcValue);
            displayValue = getMotorDisplayValue();
            setSegClock(0); //모터 모드: seg_clock (:) 끄기
        }
        else if(currentMode == MODE_MOTOR_Minus) {
            GPIOD_PSOR |= (1 << PTD15); //red off
            GPIOD_PSOR |= (1 << PTD16); //green off
            //모터 모드 (-)
            setMotorDirectionMinus();
            adc_start();
            adcValue = read_adc_chx();
            update_pwm_from_adc(adcValue);
            displayValue = getMotorDisplayValue();
            setSegClock(0); //모터 모드: seg_clock (:) 끄기
        }
        else if(currentMode == MODE_TAXI) {
            //택시 미터기 모드
            displayValue = taxiMeterValue;
            setSegClock(0); //(:) 끄기
        }
        //idle 모드가 아닐 때에만 세그먼트 디스플레이 갱신
        if(currentMode != MODE_IDLE) {
		    display6DigitNumber(displayValue); //지속적으로 세그먼트 숫자 표시
	    }
    }
}

//타이머 인터럽트 설정
void LPIT0_init(void)
{
	PCC_LPIT &= ~((0b111) << PCS_BITS);
	PCC_LPIT |= ((0b110) << PCS_BITS);
	PCC_LPIT |= (1 << CGC_BIT);

	LPIT_MCR |= (1 << M_CEN_BIT);

	LPIT_MIER |= (1 << TIE0_BIT);

	LPIT_TVAL0 = 400000;	//기본 40000000hz = 40mhz / 400000 = 100hz = 10ms 주기

	LPIT_TCTRL0 &= ~((0b11)<<MODE_BITS);
	LPIT_TCTRL0 |= (1<<T_EN_BIT);
}

//인터럽트 작동 코드 (타이머 사용)
void LPIT0_Ch0_IRQHandler(void)
{
	//스톱워치 모드를 먼저 전제로 물어봄
    if(currentMode == MODE_TIMER) {
        //스톱워치 모드
        if (isTimerRunning) {
            //10ms마다 카운터 증가
	        Counter++;

	        if(Counter >= 1) {
		        Counter = 0;
		        timerCounter++;  // 0.01 단위 증가

		        //오버플로우 방지 (9999.99 초과 시 리셋)
		        if((baseValue + timerCounter) >= 1000000) {
			        baseValue = 0;
			        timerCounter = 0;
		        }
	        }
        }
    }

    else if(currentMode == MODE_CLOCK) {
        //시계 모드
        //정확히 23:59.59를 초과해야 00:00.00으로 초기화
        if (isTimerRunning) {
            Counter++;

            if(Counter >= 100) {
                Counter = 0;
                clockSeconds++; //초 증가
                //초가 60이 되면 분 증가
                if(clockSeconds >= 60) {
                    clockSeconds = 0;
                    clockMinutes++;
                //분이 60이 되면 시 증가
                    if(clockMinutes >= 60) {
                        clockMinutes = 0;
                        clockHours++;
                        //시가 24가 되면 0시로 초기화
                        if(clockHours >= 24) {
                            clockHours = 0;
                        }
                    }
                }
            }
        }
    }

    //택시 미터기 모드를 위한 로직
    if(currentMode == MODE_TAXI && taxiModeActive) {
        //신호등 상태에 따른 모터 제어 + 그와 연계된 미터기 증가
        if (signalState == SIGNAL_RED) {
            //빨간불, 정지 상태 (공회전)
            FTM2_C0V = 0;
            FTM2_C1V = 0;
            taxiMeterValue += 3; //공회전: 초당 3.00 증가 (10ms 주기 × 300 = 3.00초)
        } else if (signalState == SIGNAL_YELLOW) {
            //노란불, 자동으로 감속하는 중
            //2초 동안 점진적으로 0으로 감소 (선형)
            // dutyAtYellow에서 시작하여 매끄럽게 감속
            int fallingTime = signalTimer;

            // dutyAtYellow가 유효한 값인지 확인 (0이면 초록불에서의 속도 사용)
            int startDuty;
            if(dutyAtYellow > 0) {
                startDuty = dutyAtYellow;
            } else {
                startDuty = motorDuty;
            }
            int autoReducedDuty = (startDuty * (SIGNAL_YELLOW_TIME - fallingTime)) / SIGNAL_YELLOW_TIME;

            // 노란불 상태에서도 스위치로 추가 제어 가능 (manual 제어)
            int sw3_pressed;
            int sw4_pressed;

            // SW3 (브레이크) 상태 확인
            if((GPIOC_PDIR & (1 << PTC12)) == 0) {
                sw3_pressed = 1;  // 눌림
            } else {
                sw3_pressed = 0;  // 안 눌림
            }

            // SW4 (액셀) 상태 확인
            if((GPIOC_PDIR & (1 << PTC13)) == 0) {
                sw4_pressed = 1;  // 눌림
            } else {
                sw4_pressed = 0;  // 안 눌림
            }

            if(sw3_pressed) {
                // 브레이크 누르면 더 빨리 감속
                if(autoReducedDuty > 100) {
                    autoReducedDuty = autoReducedDuty - 100;
                } else {
                    autoReducedDuty = 0;
                }
            }

            motorDuty = autoReducedDuty;
            UpdateMotorPWM();

            //미터기는 현재 속도에 비례해서 증가
            int rate = (motorDuty * (300 - 1)) / 999; //0~1000 -> 0~300 비율
            taxiMeterValue += rate;
        } else if(signalState == SIGNAL_GREEN) {
            //초록불, 정상 주행 및 감속 자유롭게 가능
            //motorDuty는 스위치 3, 4로 폴링 방식으로 제어함
            UpdateMotorDuty();

            //미터기 증가 로직
            if(motorDuty <= 0) {
                //완전 정지 상태 (duty = 0): 빨간불과 동일하게 공회전 요금 부과
                taxiMeterValue += 3; //공회전: 초당 3.00 증가 (10ms 주기 × 300 = 3.00초)
            } else {
                //주행 중: 현재 속도에 비례해서 증가
                int rate = (motorDuty * (300 - 1)) / 999; //0~1000 -> 0~300 비율
                taxiMeterValue += rate;
            }

            //오버플로 처리
            if(taxiMeterValue >= 1000000) {
                taxiMeterValue = 0; //1000000 이상이면 0으로 초기화
            }
        }

        //신호등 상태 전환 타이머 업데이트
        signalTimer++;
        if(signalTimer >= signalDuration) {
            switch(signalState) {
                case SIGNAL_GREEN:
                    SetSignalLight(SIGNAL_YELLOW);
                    UpdateTaxiLCD();
                    break;
                case SIGNAL_YELLOW:
                    SetSignalLight(SIGNAL_RED);
                    UpdateTaxiLCD();
                    break;
                case SIGNAL_RED:
                    motorDuty = CreepDuty; //600에서부터 시작
                    SetSignalLight(SIGNAL_GREEN);
                    UpdateTaxiLCD();
                    break;
            }
        }
    }
	//인터럽트 플래그 클리어
	LPIT_MSR |= (1 << TIF0_BIT);
}

//========================================
// 인터럽트 핸들러: PORT A (SW1, SW2, SW5)
//========================================
void PORTA_IRQHandler(void)
{
    //========================================
    // SW1 (PTA12): 리셋/RPM/초 증가
    //========================================
    if(PORTA_PCR12 & (1 << ISF_BIT)) {
        if (currentMode == MODE_IDLE) {
            //idle 모드에서는 아무 동작 안함
        } else if(currentMode == MODE_TAXI) {
            taxiMeterValue = 0; //미터기 0으로 설정
            UpdateTaxiLCD(); //택시 모드를 위한 lcd를 만들었으므로 이걸 이용
        } else if(currentMode == MODE_MOTOR_Minus)
        {   motorUnit = MOTOR_UNIT_RPM; //sw1: rpm 모드 설정
            Set_LCD_Line1("Motor-Unit:RPM");
        } else if(currentMode == MODE_MOTOR_Plus) {
            motorUnit = MOTOR_UNIT_RPM; //sw1: rpm 모드 설정
            Set_LCD_Line1("Motor+Unit:RPM");
        } else if(currentMode == MODE_TIMER) {
                //타이머 모드: 000000으로 설정
                setBaseValue(0);
        } else if (currentMode == MODE_CLOCK) {
            //시계 모드: 1초 증가 (digit 1, 2 증가)
            clockSeconds++;
            if(clockSeconds >= 60) {
                clockSeconds = 0;
                clockMinutes++;
                if(clockMinutes >= 60) {
                    clockMinutes = 0;
                    clockHours++;
                    if(clockHours >= 24) {
                        clockHours = 0;
                        }
                    }
                }
            }
        PORTA_PCR12 |= (1 << ISF_BIT);
    }

    //========================================
    // SW2 (PTA13): 방향 토글/MPS/분 증가
    //========================================
    if(PORTA_PCR13 & (1 << ISF_BIT)) {
        if (currentMode == MODE_IDLE) {
            //idle 모드에서는 아무 동작 안함
        } else if(currentMode == MODE_TAXI) {
            isMotorForward = !isMotorForward; //토글 형식으로 진행방향 정반대로 바꿈
            UpdateMotorPWM(); //모터 방향
            UpdateTaxiLCD(); //택시 모드를 위한 lcd를 만들었으므로 이걸 이용
        } else if(currentMode == MODE_MOTOR_Minus) {
            motorUnit = MOTOR_UNIT_MPS; //sw2: m/s 모드 설정
            Set_LCD_Line1("Motor-Unit:cmps");
        } else if(currentMode == MODE_MOTOR_Plus) {
            motorUnit = MOTOR_UNIT_MPS; //sw2: m/s 모드 설정
            Set_LCD_Line1("Motor+Unit:cmps");
        } else if(currentMode == MODE_TIMER) {
            //타이머 모드: 300000으로 설정 (3000.00)
            setBaseValue(300000);
        } else {
            //시계 모드: 1분 증가 (digit 3, 4 증가)
            clockMinutes++;
            if(clockMinutes >= 60) {
                clockMinutes = 0;
                clockHours++;
                if(clockHours >= 24) {
                    clockHours = 0;
                    }
                }
            }
        PORTA_PCR13 |= (1 << ISF_BIT);
    }

    //========================================
    // SW5 (PTA14): 모드 전환 (IDLE→TIMER→CLOCK→MOTOR+→MOTOR-→TAXI→IDLE)
    //========================================
    if(PORTA_PCR14 & (1 << ISF_BIT)) {
        // IDLE → TIMER
        if (currentMode == MODE_IDLE) {
            currentMode = MODE_TIMER;
            setSegClock(0);  //콜론 끄기 (타이머 모드)
            if(isTimerRunning) {
                Set_LCD_Line1("Stopwatch:Run");
            } else {
                Set_LCD_Line1("Stopwatch:Stop");
            }
            taxiModeActive = 0;

        // TIMER → CLOCK
        } else if(currentMode == MODE_TIMER) {
            currentMode = MODE_CLOCK;
            setSegClock(1);  //콜론 켜기 (시계 모드)
            if(isTimerRunning) {
                Set_LCD_Line1("Clock:Run");
            } else {
                Set_LCD_Line1("Clock:Stop");
            }
            taxiModeActive = 0;

        // CLOCK → MOTOR+
        } else if (currentMode == MODE_CLOCK) {
            currentMode = MODE_MOTOR_Plus;
            setSegClock(0);  //콜론 끄기 (모터 모드)
            isTimerRunning = 1; //segment 작동 (모터 속도값을 측정해야 하므로)
            switch (motorUnit) {
                case MOTOR_UNIT_RPM:
                    Set_LCD_Line1("Motor+Unit:RPM");
                    break;
                case MOTOR_UNIT_MPS:
                    Set_LCD_Line1("Motor+Unit:cmps");
                    break;
                case MOTOR_UNIT_PERCENT:
                    Set_LCD_Line1("Motor+Unit:Duty");
                    break;
                case MOTOR_UNIT_VOLT:
                    Set_LCD_Line1("Motor+Unit:Vin");
                    break;
            }
            taxiModeActive = 0;

        // MOTOR+ → MOTOR-
        } else if (currentMode == MODE_MOTOR_Plus) {
            currentMode = MODE_MOTOR_Minus;
            setSegClock(0);  //콜론 끄기 (모터 모드)
            isTimerRunning = 1; //segment 작동 (모터 속도값을 측정해야 하므로)
            switch (motorUnit) {
                    case MOTOR_UNIT_RPM:
                        Set_LCD_Line1("Motor-Unit:RPM");
                        break;
                    case MOTOR_UNIT_MPS:
                        Set_LCD_Line1("Motor-Unit:cmps");
                        break;
                    case MOTOR_UNIT_PERCENT:
                        Set_LCD_Line1("Motor-Unit:Duty");
                        break;
                    case MOTOR_UNIT_VOLT:
                        Set_LCD_Line1("Motor-Unit:Vin");
                        break;
                }
            taxiModeActive = 0;

        // MOTOR- → TAXI
        } else if (currentMode == MODE_MOTOR_Minus) {
            currentMode = MODE_TAXI;
            setSegClock(0); //콜론 끄기 (택시 모드)
            isTimerRunning = 1; //미터기 상승
            taxiModeActive = 1; //신호등 자동 전환 활성화
            UpdateTaxiLCD(); //택시 모드를 위한 lcd를 만들었으므로 이걸 이용
            SetSignalLight(SIGNAL_GREEN); //초록불로 시작

        // TAXI → IDLE
        } else if (currentMode == MODE_TAXI) {
            currentMode = MODE_IDLE;
            setSegClock(0);  //콜론 끄기 (타이머 모드)
            isTimerRunning = 0;
            Set_LCD_Line1("Mode:Idle");
            taxiModeActive = 0; //신호등 자동전환 종료
        }
        Counter = 0;
        PORTA_PCR14 |= (1 << ISF_BIT);
    }
}

//========================================
// 인터럽트 핸들러: PORT C (SW3, SW4)
//========================================
void PORTC_IRQHandler(void)
{
    //========================================
    // SW3 (PTC12): Duty/값 설정/시간 증가
    //========================================
    if(PORTC_PCR12 & (1 << ISF_BIT)) {
        if(currentMode != MODE_TAXI && currentMode != MODE_IDLE) {
            if(currentMode == MODE_MOTOR_Minus) {
                motorUnit = MOTOR_UNIT_PERCENT; //sw3: % 표시
                Set_LCD_Line1("Motor-Unit:Duty");
            } else if(currentMode == MODE_MOTOR_Plus) {
                motorUnit = MOTOR_UNIT_PERCENT; //sw3: % 표시
                Set_LCD_Line1("Motor+Unit:Duty");
            } else if(currentMode == MODE_TIMER) {
                //타이머 모드: 600000으로 설정 (6000.00)
                setBaseValue(600000);
            } else {
                //시계 모드: 1시간 증가 (digit 5, 6 증가)
                clockHours++;
                if(clockHours >= 24) {
                    clockHours = 0;
                }
            }
        }
        // 항상 플래그 클리어 (택시 모드도 포함)
        PORTC_PCR12 |= (1 << ISF_BIT);
    }

    //========================================
    // SW4 (PTC13): Volt/스톱워치-시계 토글
    //========================================
    if(PORTC_PCR13 & (1 << ISF_BIT)) {
        if(currentMode != MODE_TAXI && currentMode != MODE_IDLE) {
            if(currentMode == MODE_MOTOR_Minus) {
                motorUnit = MOTOR_UNIT_VOLT; //sw4: 전압 표시
                Set_LCD_Line1("Motor-Unit:Vin");
            } else if (currentMode == MODE_MOTOR_Plus) {
                motorUnit = MOTOR_UNIT_VOLT; //sw4: 전압 표시
                Set_LCD_Line1("Motor+Unit:Vin");
            } else if (currentMode == MODE_TIMER) {
                isTimerRunning = !isTimerRunning;
                if (isTimerRunning) {
                    Set_LCD_Line1("Stopwatch:Run");
                } else {
                    Set_LCD_Line1("Stopwatch:Stop");
                }
            } else if (currentMode == MODE_CLOCK) {
                isTimerRunning = !isTimerRunning;
                if (isTimerRunning) {
                    Set_LCD_Line1("Clock:Run");
                } else {
                    Set_LCD_Line1("Clock:Stop");
                }
            }
        }
        // 항상 플래그 클리어 (택시 모드도 포함)
        PORTC_PCR13 |= (1 << ISF_BIT);
    }
}


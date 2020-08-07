#ifndef __TIECAP_H
#define __TIECAP_H

/* ECAP regs and bits */
#define CAP1			0x08
#define CAP2			0x0c
#define ECCTL1			0x28
#define ECCTL1_RUN_FREE		BIT(15)
#define ECCTL1_CAPLDEN		BIT(8)
#define ECCTL1_CAP2POL		BIT(2)
#define ECCTL1_CTRRST1		BIT(1)
#define ECCTL1_CAP1POL		BIT(0)
#define ECCTL2			0x2a
#define ECCTL2_SYNCO_SEL_DIS	BIT(7)
#define ECCTL2_TSCTR_FREERUN	BIT(4)
#define ECCTL2_REARM		BIT(3)
#define ECCTL2_STOP_WRAP_2	BIT(1)
#define ECEINT			0x2c
#define ECFLG			0x2e
#define ECCLR			0x30
#define ECINT_CTRCMP		BIT(7)
#define ECINT_CTRPRD		BIT(6)
#define ECINT_CTROVF		BIT(5)
#define ECINT_CEVT4		BIT(4)
#define ECINT_CEVT3		BIT(3)
#define ECINT_CEVT2		BIT(2)
#define ECINT_CEVT1		BIT(1)
#define ECINT_ALL		(ECINT_CTRCMP |	\
				ECINT_CTRPRD |	\
				ECINT_CTROVF |	\
				ECINT_CEVT4 |	\
				ECINT_CEVT3 |	\
				ECINT_CEVT2 |	\
				ECINT_CEVT1)

/* ECAP driver flags */
#define ECAP_POLARITY_HIGH	BIT(1)
#define ECAP_ENABLED		BIT(0)

#endif
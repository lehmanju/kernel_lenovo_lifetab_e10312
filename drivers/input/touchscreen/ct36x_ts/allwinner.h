
#ifndef ALLWINNER_H
#define ALLWINNER_H


#define POSITIVE_EDGE			0x0
#define NEGATIVE_EDGE			0x1
#define HIGH_LEVEL			0x2
#define LOW_LEVEL			0x3
#define DOUBLE_EDGE			0x4

// gpio base address
#define PIO_BASE_ADDRESS             (0x01C20800)
#define PIO_RANGE_SIZE               (0x400)

#define IRQ_EINT0	(0)
#define IRQ_EINT1	(1)
#define IRQ_EINT2	(2)
#define IRQ_EINT3	(3)
#define IRQ_EINT4	(4)
#define IRQ_EINT5	(5)
#define IRQ_EINT6	(6)
#define IRQ_EINT7	(7)
#define IRQ_EINT8	(8)
#define IRQ_EINT9	(9)
#define IRQ_EINT10	(10)
#define IRQ_EINT11	(11)
#define IRQ_EINT12	(12)
#define IRQ_EINT13	(13)
#define IRQ_EINT14	(14)
#define IRQ_EINT15	(15)
#define IRQ_EINT16	(16)
#define IRQ_EINT17	(17)
#define IRQ_EINT18	(18)
#define IRQ_EINT19	(19)
#define IRQ_EINT20	(20)
#define IRQ_EINT21	(21)
#define IRQ_EINT22	(22)
#define IRQ_EINT23	(23)
#define IRQ_EINT24	(24)
#define IRQ_EINT25	(25)
#define IRQ_EINT26	(26)
#define IRQ_EINT27	(27)
#define IRQ_EINT28	(28)
#define IRQ_EINT29	(29)
#define IRQ_EINT30	(30)
#define IRQ_EINT31	(31)

#define PIO_INT_STAT_OFFSET		(0x214)
#define PIO_INT_CTRL_OFFSET		(0x210)
#define PIO_INT_CFG0_OFFSET		(0x200)
#define PIO_INT_CFG1_OFFSET		(0x204)
#define PIO_INT_CFG2_OFFSET		(0x208)
#define PIO_INT_CFG3_OFFSET		(0x20C)


#define CT36X_TS_IRQ_NO			(IRQ_EINT21)
#define CT36X_TS_IRQ_MODE		(NEGATIVE_EDGE)

#endif


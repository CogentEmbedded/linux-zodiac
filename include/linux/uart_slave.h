#ifndef _LINUX_UART_SLAVE_H
#define _LINUX_UART_SLAVE_H
struct uart_slave {
	struct device *tty_dev;
	struct tty_driver *tty_drv;
	struct tty_operations ops;
	struct device dev;
	bool finalized;
};

int uart_slave_add_tty(struct uart_slave *slave);
int uart_slave_driver_register(struct device_driver *drv);
#if IS_ENABLED(CONFIG_UART_SLAVE)
void uart_slave_activate(struct tty_struct *tty);
int uart_slave_register(struct device *parent,
			struct device *tty, struct tty_driver *drv);
#else
static inline void uart_slave_activate(struct tty_struct *tty)
{
}
static inline int uart_slave_register(struct device *parent,
				     struct device *tty,
				     struct tty_driver *drv)
{
	return -ENODEV;
}
#endif

#endif /* _LINUX_UART_SLAVE_H */

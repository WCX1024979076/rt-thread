idf_component_register(
	SRCS
	../../../libcpu/risc-v/common/interrupt_gcc.S
	../../../libcpu/risc-v/common/cpuport.c
	../../../libcpu/risc-v/common/context_gcc.S
	../../../libcpu/risc-v/common/trap_common.c
	../../../components/drivers/ipc/completion.c
	../../../components/drivers/ipc/dataqueue.c
	../../../components/drivers/ipc/pipe.c
	../../../components/drivers/ipc/ringblk_buf.c
	../../../components/drivers/ipc/ringbuffer.c
	../../../components/drivers/ipc/waitqueue.c
	../../../components/drivers/ipc/workqueue.c
	../../../components/drivers/misc/pin.c
	../../../components/drivers/serial/serial.c
	../drivers/drv_gpio.c
	../drivers/drv_hw_i2c.c
	../drivers/drv_uart.c
	../drivers/board.c
	../../../components/finsh/msh_parse.c
	../../../components/finsh/cmd.c
	../../../components/finsh/msh.c
	../../../components/finsh/shell.c
	../../../src/clock.c
	../../../src/components.c
	../../../src/device.c
	../../../src/idle.c
	../../../src/ipc.c
	../../../src/irq.c
	../../../src/kservice.c
	../../../src/mem.c
	../../../src/mempool.c
	../../../src/object.c
	../../../src/scheduler_up.c
	../../../src/thread.c
	../../../src/timer.c
	main.c

	INCLUDE_DIRS
	../../../libcpu/risc-v/common
	../../../components/drivers/include
	../drivers
	../../../components/finsh
	..
	../../../include
)

target_link_libraries(${COMPONENT_LIB}
	INTERFACE "-u __rt_init_rt_work_sys_workqueue_init"
	INTERFACE "-u __rt_init_rt_hw_pin_init"
	INTERFACE "-u __rt_init_rt_hw_i2c_init"
	INTERFACE "-u __rt_init_rt_hw_uart_init"
	INTERFACE "-u __rt_init_finsh_system_init"
)

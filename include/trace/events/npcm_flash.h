#undef TRACE_SYSTEM
#define TRACE_SYSTEM npcm_flash

#if !defined(_TRACE_NPCM_FLASH_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_NPCM_FLASH_H

#include <linux/tracepoint.h>

TRACE_EVENT(npcm_flash,
	TP_PROTO(__u8 command, __u32 address, __u16 length),
	TP_ARGS(command, address, length),
	TP_STRUCT__entry(
		__field(__u8,	command		)
		__field(__u32,	address		)
		__field(__u16,	length		)	),

	TP_fast_assign(
		__entry->command = command;
		__entry->address = address;
		__entry->length = length;
		),
	TP_printk("cmd=%d addr=0x%x len=0x%x",
		__entry->command, __entry->address, __entry->length));

#endif /* _TRACE_NPCM_FLASH_H */

/* This part must be outside protection */
#include <trace/define_trace.h>

OUTPUT_ARCH(mips)
ENTRY(start)
MEMORY{
	/* TOTAL MEM :256KBytes */

	/*160Kbytes, .text, .data.*/
	voice_mem(rw): ORIGIN = 0x8FF00000, LENGTH = 0x28000
	/*96Kbytes: .resource.*/
	voice_res(rw): ORIGIN = 0x8FF28000, LENGTH = 0x18000
}

SECTIONS
{
	.voice_mem : {
		. = ALIGN(4);
			*(.text)
			/**(.data)*/

			/**(.bss)*/
	} > voice_mem

	.voice_res : {

	} > voice_res

	/DISCARD/ : {
		*(COMMON)
		*(.pdri)
		*(.comment)
		*(.gnu.attributes)
		*(.reginfo)
	}
}

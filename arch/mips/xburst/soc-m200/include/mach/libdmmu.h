#ifndef _LIBDMMU_
#define _LIBDMMU_

int dmmu_map(unsigned long vaddr,unsigned long len);
int dmmu_unmap(unsigned long vaddr, int len);

#endif

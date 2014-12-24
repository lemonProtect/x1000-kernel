


#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/debugfs.h>
#include <linux/mempolicy.h>
#include <linux/mm_types.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/current.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>
#include <asm/io.h>
#include <linux/vmalloc.h>
#include <linux/sched.h>
#include <jz_proc.h>

#include <mach/libdmmu.h>

#define DMMU_PGD_VLD 0x1
#define KSEG0_LOW_LIMIT		0x80000000
#define KSEG1_HEIGH_LIMIT	0xC0000000

LIST_HEAD(dmmu_list);

struct dmmu_map_node {
	unsigned int count;
	unsigned long index;
	unsigned long start;
	unsigned long end;
	unsigned long page;
	struct list_head list;
};

struct dmmu_handle {
	pid_t tgid;
	unsigned long pdg;
	struct list_head node_list;
	struct list_head list;
};

static unsigned int get_paddr(unsigned int vaddr)
{
	unsigned int addr = vaddr & (PAGE_SIZE-1);
	pgd_t *pgdir;
	pmd_t *pmdir;
	pte_t *pte;
	pgdir = pgd_offset(current->mm, vaddr);
	if(pgd_none(*pgdir) || pgd_bad(*pgdir))
		return 0;
	pmdir = pmd_offset((pud_t *)pgdir, vaddr);
	if(pmd_none(*pmdir) || pmd_bad(*pmdir))
		return 0;
	pte = pte_offset(pmdir,vaddr);
	if (pte_present(*pte)) {
		return addr | (pte_pfn(*pte) << PAGE_SHIFT);
	}
	return 0;
}


static int dmmu_v2p(unsigned long vaddr)
{
	if(vaddr < KSEG0_LOW_LIMIT)
		return get_paddr(vaddr);
	
	if(vaddr >= KSEG0_LOW_LIMIT && vaddr < KSEG1_HEIGH_LIMIT)
		return virt_to_phys((void *)vaddr);

	panic("dmmu_v2p error!");
	return 0;
}

static void unmap_node(struct dmmu_map_node *n,int checkcount)
{
	if(checkcount && --n->count)
		return;

	free_page(n->page);
	list_del(&n->list);
	kfree(n);
}

static unsigned long map_node(struct dmmu_map_node *n,unsigned int vaddr,unsigned int end)
{
	unsigned int *pte = (unsigned int *)n->page;
	int index = ((vaddr & 0x3ff000) >> 12);
	n->start = vaddr;
	while(index < 1024 && vaddr < end) {
		pte[index++] = dmmu_v2p(vaddr);
		vaddr += 4096;
	}
	n->end = index < 1024 ? end : vaddr;
	n->count++;
	return vaddr;
}

static struct dmmu_map_node *find_node(struct dmmu_handle *handle,unsigned int vaddr)
{
	struct list_head *pos, *next;
	struct dmmu_map_node *n;

	list_for_each_safe(pos, next, &handle->node_list) {
		n = list_entry(pos, struct dmmu_map_node, list);
		if(n->index == (vaddr & 0xfffff000))
			return n;
	}
	return NULL;
}

static struct dmmu_map_node *add_node(struct dmmu_handle *handle,unsigned int vaddr)
{
	unsigned long *pgd = (unsigned long *)handle->pdg;
	struct dmmu_map_node *n = kmalloc(sizeof(*n),GFP_KERNEL);
	INIT_LIST_HEAD(&n->list);
	n->count = 0;
	n->index = vaddr & 0xffc00000;
	n->page = __get_free_page(GFP_KERNEL);
	SetPageReserved(virt_to_page((void *)n->page));
	list_add(&n->list, &handle->node_list);

	pgd[vaddr>>22] = dmmu_v2p(n->page) | DMMU_PGD_VLD;
	return n;
}

static struct dmmu_handle *find_handle(void)
{
	struct list_head *pos, *next;
	struct dmmu_handle *handle;

	list_for_each_safe(pos, next, &dmmu_list) {
		handle = list_entry(pos, struct dmmu_handle, list);
		if(handle->tgid == current->tgid)
			return handle;
	}
	return NULL;
}

static struct dmmu_handle *create_handle(void)
{
	struct dmmu_handle *handle;

	handle = kmalloc(sizeof(struct dmmu_handle),GFP_KERNEL);
	if(!handle)
		return NULL;

	handle->tgid = current->tgid;
	handle->pdg = __get_free_page(GFP_KERNEL);
	SetPageReserved(virt_to_page((void *)handle->pdg));

	if(!handle->pdg) {
		kfree(handle);
		return NULL;
	}

	INIT_LIST_HEAD(&handle->list);
	INIT_LIST_HEAD(&handle->node_list);
	list_add(&handle->list, &dmmu_list);

	return handle;
}

static int dmmu_make_present(unsigned long addr,unsigned long end)
{
	int ret, len, write;
	struct vm_area_struct * vma;
	unsigned long vm_page_prot;

	vma = find_vma(current->mm, addr);
	if (!vma)
		return -1;
	write = (vma->vm_flags & VM_WRITE) != 0;
	BUG_ON(addr >= end);
	BUG_ON(end > vma->vm_end);

	vm_page_prot = pgprot_val(vma->vm_page_prot);
	vma->vm_page_prot = __pgprot(vm_page_prot | _PAGE_VALID| _PAGE_ACCESSED | _PAGE_PRESENT);                                                        

	len = DIV_ROUND_UP(end, PAGE_SIZE) - addr/PAGE_SIZE;
	ret = get_user_pages(current, current->mm, addr,
			len, write, 0, NULL, NULL);
	vma->vm_page_prot = __pgprot(vm_page_prot);      
	if (ret < 0)
		return ret;
	return ret == len ? 0 : -1;
}

static void dmmu_cache_wback(struct dmmu_handle *handle)
{
	struct list_head *pos, *next;
	struct dmmu_map_node *n;

	dma_cache_wback(handle->pdg,PAGE_SIZE);

	list_for_each_safe(pos, next, &handle->node_list) {
		n = list_entry(pos, struct dmmu_map_node, list);
		dma_cache_wback(n->page,PAGE_SIZE);
	}
}

unsigned long dmmu_map(unsigned long vaddr,unsigned long len)
{
	int end = vaddr + len;
	struct dmmu_handle *handle;
	struct dmmu_map_node *node;

	handle = find_handle();
	if(!handle)
		handle = create_handle();
	if(!handle)
		return 0;

	if(dmmu_make_present(vaddr,vaddr+len))
		return 0;

	while(vaddr < end) {
		node = find_node(handle,vaddr);
		if(!node) {
			node = add_node(handle,vaddr);
		}

		vaddr = map_node(node,vaddr,end);
	}

	dmmu_cache_wback(handle);

	return handle->pdg;
}

int dmmu_unmap(unsigned long vaddr, int len)
{
	unsigned long end = vaddr + len;
	struct dmmu_handle *handle;
	struct dmmu_map_node *node;

	handle = find_handle();
	if(!handle)
		return 0;

	while(vaddr < end) {
		node = find_node(handle,vaddr);
		if(node)
			unmap_node(node,1);
	}
	
	if(list_empty(&handle->node_list)) {
		list_del(&handle->list);
		free_page(handle->pdg);
		kfree(handle);
	}

	return 0;
}

int dmmu_unmap_all(void)
{
	struct dmmu_handle *handle;
	struct dmmu_map_node *node;
	struct list_head *pos, *next;

	handle = find_handle();
	if(!handle)
		return 0;
	
	list_for_each_safe(pos, next, &handle->node_list) {
		node = list_entry(pos, struct dmmu_map_node, list);
		unmap_node(node,0);
	}
	
	list_del(&handle->list);
	free_page(handle->pdg);
	kfree(handle);

	return 0;
}

static void dmmu_dump_handle(struct seq_file *m, void *v, struct dmmu_handle *h)
{
	struct list_head *pos, *next;
	struct dmmu_map_node *n;

	printk("tgid %d ======================================================\n",h->tgid);
	list_for_each_safe(pos, next, &h->node_list) {
		n = list_entry(pos, struct dmmu_map_node, list);
		{
			int i = 0;
			int vaddr = n->start;
			unsigned int *pte = (unsigned int *)n->page;

			while(vaddr <= n->end) {
				if(i++%8 == 0)
					printk("\nvaddr %08x : ",vaddr & 0xfffff000);
				printk("%08x ",pte[(vaddr & 0x3ff000)>>12]);
				vaddr += 4096;
			}
			printk("\n\n");
		}
	}
}

static int dmmu_proc_show(struct seq_file *m, void *v)
{
	struct list_head *pos, *next;
	struct dmmu_handle *handle;

	list_for_each_safe(pos, next, &dmmu_list) {
		handle = list_entry(pos, struct dmmu_handle, list);
			dmmu_dump_handle(m, v, handle);
	}

	return 0;
}

static int dmmu_open(struct inode *inode, struct file *file)
{
	return single_open(file, dmmu_proc_show, PDE_DATA(inode));
}

static const struct file_operations dmmus_proc_fops ={
	.read = seq_read,
	.open = dmmu_open,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init init_dmmu_proc(void)
{
	struct proc_dir_entry *p;
	p = jz_proc_mkdir("dmmu");
	if (!p) {
		pr_warning("create_proc_entry for common dmmu failed.\n");
		return -ENODEV;
	}
	proc_create("dmmus", 0600,p,&dmmus_proc_fops);

	return 0;
}

module_init(init_dmmu_proc);

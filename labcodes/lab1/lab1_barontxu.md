

#Lab1 
### 计23 徐梓哲 2012011487
#      

##1.
###ucoreimg的创建
	create ucore.img
	UCOREIMG	:= $(call totarget,ucore.img)

	$(UCOREIMG): $(kernel) $(bootblock)
		$(V)dd if=/dev/zero of=$@ count=10000
		$(V)dd if=$(bootblock) of=$@ conv=notrunc
		$(V)dd if=$(kernel) of=$@ seek=1 conv=notrunc

	$(call create_target,ucore.img)
我们可以看到需要bootblock 以及kernel的生成。

###bootblock的生成
> ###create bootblock	
bootfiles = $(call listf_cc,boot)
$(foreach f,$(bootfiles),$(call cc_compile,$(f),$(CC),$(CFLAGS) -Os -nostdinc))   
bootblock = $(call totarget,bootblock) 	
$(bootblock): $(call toobj,$(bootfiles)) | $(call totarget,sign)	
	@echo + ld $@	
	$(V)$(LD) $(LDFLAGS) -N -e start -Ttext 0x7C00 $^ -o $(call toobj,bootblock)	
	@$(OBJDUMP) -S $(call objfile,bootblock) > $(call asmfile,bootblock)	
	@$(OBJCOPY) -S -O binary $(call objfile,bootblock) $(call outfile,bootblock)	
	@$(call totarget,sign) $(call outfile,bootblock) $(bootblock)	
	$(call create_target,bootblock) 		
>>	这时候还需要生成bootasm.o、bootmain.o、sign	（ld -m    elf_i386 -nostdlib -N -e start -Ttext 0x7C00 obj/boot/bootasm.o obj/boot/bootmain.o -o obj/bootblock.o）
	bootasm.o、bootmain.o由以下代码生成		
	bootfiles = $(call listf_cc,boot) 	   
			$(foreach f,$(bootfiles),$(call cc_compile,$(f),$(CC),\     	
			$(CFLAGS) -Os -nostdinc))       
	gcc -Iboot/ -fno-builtin -Wall -ggdb -m32 -gstabs \              
			-nostdinc  -fno-stack-protector -Ilibs/ -Os -nostdinc \            
			-c boot/bootasm.S -o obj/boot/bootasm.o        
	bootmain.o 由 bootmain.c 生成代码为
	gcc -Iboot/ -fno-builtin -Wall -ggdb -m32 -gstabs -nostdinc \       
		-fno-stack-protector -Ilibs/ -Os -nostdinc \     
		-c boot/bootmain.c -o obj/boot/bootmain.o      
	生成sign工具的makefile代码为  // ????????????????
 	$(call add_files_host,tools/sign.c,sign,sign)          
	 $(call create_target_host,sign,sign)     	      
	make "V="显示为            
	gcc -Itools/ -g -Wall -O2 -c tools/sign.c \         
	-o obj/sign/tools/sign.o            
	gcc -g -Wall -O2 obj/sign/tools/sign.o -o bin/sign      // ????????????????     
	
###bin/kernel              
>	生成kernel的代码为              
		$(kernel): tools/kernel.ld              
		$(kernel): $(KOBJS)              
		@echo + ld $@              
		$(V)$(LD) $(LDFLAGS) -T tools/kernel.ld -o $@ $(KOBJS)                            
		@$(OBJDUMP) -S $@ > $(call asmfile,kernel)              
		@$(OBJDUMP) -t $@ | $(SED) '1,/SYMBOL TABLE/d; s/ .* / /; \              
			/^$$/d' > $(call symfile,kernel)              
		
		kernel需要 kernel.ld init.o readline.o stdio.o kdebug.o              
		kmonitor.o panic.o clock.o console.o intr.o picirq.o trap.o              
		trapentry.o vectors.o pmm.o  printfmt.o string.o              
		kernel.ld已存在                
		obj/kern/*/*.o               
		生成这些.o文件的相关makefile代码为              
		$(call add_files_cc,$(call listf_cc,$(KSRCDIR)),kernel,\              
			$(KCFLAGS))
		对obj/kern/init/init.o              
		编译需要init.c              
		实际命令为              
		gcc -Ikern/init/ -fno-builtin -Wall -ggdb -m32 -gstabs -nostdinc  -fno-stack-protector -Ilibs/ -Ikern/debug/ -Ikern/driver/ -Ikern/trap/ -Ikern/mm/ -c kern/init/init.c -o obj/kern/init/init.o
		其余类似
	
	
>
	生成一个有10000个块的文件，每个块默认512字节，用0填充
	dd if=/dev/zero of=bin/ucore.img count=10000
	把bootblock中的内容写到第一个块
	dd if=bin/bootblock of=bin/ucore.img conv=notrunc
	从第二个块开始写kernel中的内容
	dd if=bin/kernel of=bin/ucore.img seek=1 conv=notrunc

[练习1.2] 一个被系统认为是符合规范的硬盘主引导扇区的特征是什么?

一个磁盘主引导扇区只有512字节。且第510个字节是0x55，第511个字节是0xAA。




##2.
1.删除tools/gdbinit中最后的continue,可以第一条指令开始时断掉程序。

2.在tools/gdbinit 加入
	b *0x7c00
    c

可以断在想要的位置。

3.通过x /100i $pc查看命令，得到代码一致

4.在kern_init设置断点，通过x /i $pc查看指令，让程序一步步进行，可以看到程序执行了所有命令，安全运行。

	
	
	
##3.
首先Set up the important data segment registers (DS, ES, SS).        
然后使能A20           
	
	为何开启A20？	   
	A20禁止时，1mb以上的地址不可访问。在A20被置为高电平后，可以使用更大的内存。
	开启方法：
	seta20.1:
    inb $0x64, %al                                  # Wait for not busy(8042 input buffer empty).
    testb $0x2, %al
    jnz seta20.1
    movb $0xd1, %al                                 # 0xd1 -> port 0x64
    outb %al, $0x64                                 # 0xd1 means: write data to 8042's P2 port
	seta20.2:
    	inb $0x64, %al                                  # Wait for not busy(8042 input buffer 	empty).
    	testb $0x2, %al
	    jnz seta20.2
	    movb $0xdf, %al                                 # 0xdf -> port 0x60
    	outb %al, $0x60                                 # 0xdf = 11011111, means set P2's A20 	bit(the 1 bit) to 1


然后初始化GDT表：        
	    lgdt gdtdesc

然后进入保护模式：将cr0寄存器PE位置1
	   
	    movl %cr0, %eax
	    orl $CR0_PE_ON, %eax
	    movl %eax, %cr0

再 # Jump to next instruction, but in 32-bit code segment.          
    # Switches processor into 32-bit mode.
    ljmp $PROT_MODE_CSEG, $protcseg
    
再 # Set up the protected-mode data segment registers         
最后       

	#Set up the protected-mode data segment registers
    movw $PROT_MODE_DSEG, %ax                       # Our data segment selector
    movw %ax, %ds                                   # -> DS: Data Segment
    movw %ax, %es                                   # -> ES: Extra Segment
    movw %ax, %fs                                   # -> FS
    movw %ax, %gs                                   # -> GS
    movw %ax, %ss                                   # -> SS: Stack Segment
    # Set up the stack pointer and call into C. The stack region is from 0--start(0x7c00)
    movl $0x0, %ebp
    movl $start, %esp
    call bootmain
完成保护模式


##4.
首先，两个函数
也就是
  	
	
	static void
	readseg(uintptr_t va, uint32_t count, uint32_t offset) {
    	uintptr_t end_va = va + count;

    	// round down to sector boundary
	    va -= offset % SECTSIZE;
	    // translate from bytes to sectors; kernel starts at sector 1
    	uint32_t secno = (offset / SECTSIZE) + 1;
	    // If this is too slow, we could read lots of sectors at a time.
    	// We'd write more to memory than asked, but it doesn't matter --
	    // we load in increasing order.
    	for (; va < end_va; va += SECTSIZE, secno ++) {
        	readsect((void *)va, secno);
    	}
	}
	static void
	readsect(void *dst, uint32_t secno) {
    // wait for disk to be ready
    waitdisk();
    outb(0x1F2, 1);                         // count = 1
    outb(0x1F3, secno & 0xFF);
    outb(0x1F4, (secno >> 8) & 0xFF);
    outb(0x1F5, (secno >> 16) & 0xFF);
    outb(0x1F6, ((secno >> 24) & 0xF) | 0xE0);
    outb(0x1F7, 0x20);                      // cmd 0x20 - read sectors
    // wait for disk to be ready
    waitdisk();
    // read a sector
    insl(0x1F0, dst, SECTSIZE / 4);
	}

其中readsect函数从设备的第secno扇区读取数据到dst位置

	outb(0x1F2, 1);                         // 读写的扇区数为一
    outb(0x1F3, secno & 0xFF);				//LBA的0-7位
    outb(0x1F4, (secno >> 8) & 0xFF);
    outb(0x1F5, (secno >> 16) & 0xFF);
    outb(0x1F6, ((secno >> 24) & 0xF) | 0xE0);
    outb(0x1F7, 0x20);                      // cmd 0x20 - read sectors 开始读

readseg则是对readsect的调用。


然后，在MAIN函数里，先检查ELF的头部，判断是否合法，如果不合法，直接跳过到

    outw(0x8A00, 0x8A00);
    outw(0x8A00, 0x8E00);
如果合法的话，直接载入数据
	
	ph = (struct proghdr *)((uintptr_t)ELFHDR + ELFHDR->e_phoff);
    eph = ph + ELFHDR->e_phnum;
    for (; ph < eph; ph ++) {
        readseg(ph->p_va & 0xFFFFFF, ph->p_memsz, ph->p_offset);
    }

找到入口

    // call the entry point from the ELF header
    // note: does not return
    ((void (*)(void))(ELFHDR->e_entry & 0xFFFFFF))();


##5.
需要修改kern/debug中的kdebug.c中的代码     
	
	* (1) call read_ebp() to get the value of ebp. the type is (uint32_t);        
	* (2) call read_eip() to get the value of eip. the type is (uint32_t);
	这时只要
	uint32_t ebp = read_ebp();
	uint32_t eip = read_eip();
		* (3) from 0 .. STACKFRAME_DEPTH
      *    (3.1) printf value of ebp, eip
      *    (3.2) (uint32_t)calling arguments [0..4] = the contents in address (unit32_t)ebp +2 [0..4]
      *    (3.3) cprintf("\n");
      *    (3.4) call print_debuginfo(eip-1) to print the C calling function name and line number, etc.
      *    (3.5) popup a calling stackframe
      *           NOTICE: the calling funciton's return addr eip  = ss:[ebp+4]
      *                   the calling funciton's ebp = ss:[ebp]
	只要    
	int i, j;
    for (i = 0; ebp != 0 && i < STACKFRAME_DEPTH; i ++) {
        cprintf("ebp:0x%08x eip:0x%08x args:", ebp, eip);
        uint32_t *args = (uint32_t *)ebp + 2;
        for (j = 0; j < 4; j ++)
        {
            cprintf("0x%08x ", args[j]);
        }
        cprintf("\n");
        print_debuginfo(eip - 1);
        eip = ((uint32_t *)ebp)[1];
        ebp = ((uint32_t *)ebp)[0];
    }
	即可。

##6.
需要修改trap.c中的代码
	
	step2：
	
	     /* LAB1 YOUR CODE : STEP 2 */
     /* (1) Where are the entry addrs of each Interrupt Service Routine (ISR)?
      *     All ISR's entry addrs are stored in __vectors. where is uintptr_t __vectors[] ?
      *     __vectors[] is in kern/trap/vector.S which is produced by tools/vector.c
      *     (try "make" command in lab1, then you will find vector.S in kern/trap DIR)
      *     You can use  "extern uintptr_t __vectors[];" to define this extern variable which will be used later.
      * (2) Now you should setup the entries of ISR in Interrupt Description Table (IDT).
      *     Can you see idt[256] in this file? Yes, it's IDT! you can use SETGATE macro to setup each item of IDT
      * (3) After setup the contents of IDT, you will let CPU know where is the IDT by using 'lidt' instruction.
      *     You don't know the meaning of this instruction? just google it! and check the libs/x86.h to know more.
      *     Notice: the argument of lidt is idt_pd. try to find it!
      */
	extern uintptr_t __vectors[];    int i;    for (i = 0; i < sizeof(idt) / sizeof(struct gatedesc); i ++) {        SETGATE(idt[i], 0, GD_KTEXT, __vectors[i], DPL_KERNEL);    }	#@@@#####切换态时    SETGATE(idt[T_SWITCH_TOK], 0, GD_KTEXT, __vectors[T_SWITCH_TOK], DPL_USER);	载入表#@@@#####    lidt(&idt_pd);

step3较简单，只要记录tick的次数就可以了。
	
	
	step3：
	非常简单：
	        /* (1) After a timer interrupt, you should record this event using a global variable (increase it), such as ticks in kern/driver/clock.c
         * (2) Every TICK_NUM cycle, you can print some info using a funciton, such as print_ticks().
         * (3) Too Simple? Yes, I think so!
	
	
	
	
	
	
	
	
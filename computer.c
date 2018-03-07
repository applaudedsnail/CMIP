#include <stdio.h>
#include <stdlib.h>
#include <netinet/in.h>
#include "computer.h"
#undef mips			/* gcc already has a def for mips */

#define LAST(k,n) ((k) & ((1<<(n))-1))
#define MID(k,m,n) LAST((k)>>(m),((n)-(m)))


unsigned int endianSwap(unsigned int);

void PrintInfo (int changedReg, int changedMem);
unsigned int Fetch (int);
void Decode (unsigned int, DecodedInstr*, RegVals*);
int Execute (DecodedInstr*, RegVals*);
int Mem(DecodedInstr*, int, int *);
void RegWrite(DecodedInstr*, int, int *);
void UpdatePC(DecodedInstr*, int);
void PrintInstruction (DecodedInstr*);


/*Globally accessible Computer variable*/
Computer mips;
RegVals rVals;
int signBit;

/*
 *  Return an initialized computer with the stack pointer set to the
 *  address of the end of data memory, the remaining registers initialized
 *  to zero, and the instructions read from the given file.
 *  The other arguments govern how the program interacts with the user.
 */
void InitComputer (FILE* filein, int printingRegisters, int printingMemory,
  int debugging, int interactive) {
    int k;
    unsigned int instr;

    /* Initialize registers and memory */

    for (k=0; k<32; k++) {
        mips.registers[k] = 0;
    }
    
    /* stack pointer - Initialize to highest address of data segment */
    mips.registers[29] = 0x00400000 + (MAXNUMINSTRS+MAXNUMDATA)*4;

    for (k=0; k<MAXNUMINSTRS+MAXNUMDATA; k++) {
        mips.memory[k] = 0;
    }

    k = 0;
    while (fread(&instr, 4, 1, filein)) {
	/*swap to big endian, convert to host byte order. Ignore this.*/
        mips.memory[k] = ntohl(endianSwap(instr));
        k++;
        if (k>MAXNUMINSTRS) {
            fprintf (stderr, "Program too big.\n");
            exit (1);
        }
    }

    mips.printingRegisters = printingRegisters;
    mips.printingMemory = printingMemory;
    mips.interactive = interactive;
    mips.debugging = debugging;
}

unsigned int endianSwap(unsigned int i) {
    return (i>>24)|(i>>8&0x0000ff00)|(i<<8&0x00ff0000)|(i<<24);
}

/*
 *  Run the simulation.
 */
void Simulate () {
    char s[40];  /* used for handling interactive input */
    unsigned int instr;
    int changedReg=-1, changedMem=-1, val;
    DecodedInstr d;
    
    /* Initialize the PC to the start of the code section */
    mips.pc = 0x00400000;
    while (1) {
        if (mips.interactive) {
            printf ("> ");
            fgets (s,sizeof(s),stdin);
            if (s[0] == 'q') {
                return;
            }
        }

        /* Fetch instr at mips.pc, returning it in instr */
        instr = Fetch (mips.pc);

        printf ("Executing instruction at %8.8x: %8.8x\n", mips.pc, instr);

        /* 
	 * Decode instr, putting decoded instr in d
	 * Note that we reuse the d struct for each instruction.
	 */
        Decode (instr, &d, &rVals);

        /*Print decoded instruction*/
        PrintInstruction(&d);

        /* 
	 * Perform computation needed to execute d, returning computed value 
	 * in val 
	 */
        val = Execute(&d, &rVals);

	UpdatePC(&d,val);

        /* 
	 * Perform memory load or store. Place the
	 * address of any updated memory in *changedMem, 
	 * otherwise put -1 in *changedMem. 
	 * Return any memory value that is read, otherwise return -1.
         */
      //   printf("Right before val: %d \n", val);
        val = Mem(&d, val, &changedMem);

        /* 
	 * Write back to register. If the instruction modified a register--
	 * (including jal, which modifies $ra) --
         * put the index of the modified register in *changedReg,
         * otherwise put -1 in *changedReg.
         */
        RegWrite(&d, val, &changedReg);

        PrintInfo (changedReg, changedMem);
    }
}

/*
 *  Print relevant information about the state of the computer.
 *  changedReg is the index of the register changed by the instruction
 *  being simulated, otherwise -1.
 *  changedMem is the address of the memory location changed by the
 *  simulated instruction, otherwise -1.
 *  Previously initialized flags indicate whether to print all the
 *  registers or just the one that changed, and whether to print
 *  all the nonzero memory or just the memory location that changed.
 */
void PrintInfo ( int changedReg, int changedMem) {
    int k, addr;
    printf ("New pc = %8.8x\n", mips.pc);
    if (!mips.printingRegisters && changedReg == -1) {
        printf ("No register was updated.\n");
    } else if (!mips.printingRegisters) {
        printf ("Updated r%2.2d to %8.8x\n",
        changedReg, mips.registers[changedReg]);
    } else {
        for (k=0; k<32; k++) {
            printf ("r%2.2d: %8.8x  ", k, mips.registers[k]);
            if ((k+1)%4 == 0) {
                printf ("\n");
            }
        }
    }
    if (!mips.printingMemory && changedMem == -1) {
        printf ("No memory location was updated.\n");
    } else if (!mips.printingMemory) {
        printf ("Updated memory at address %8.8x to %8.8x\n",
        changedMem, Fetch (changedMem));
    } else {
        printf ("Nonzero memory\n");
        printf ("ADDR	  CONTENTS\n");
        for (addr = 0x00400000+4*MAXNUMINSTRS;
             addr < 0x00400000+4*(MAXNUMINSTRS+MAXNUMDATA);
             addr = addr+4) {
            if (Fetch (addr) != 0) {
                printf ("%8.8x  %8.8x\n", addr, Fetch (addr));
            }
        }
    }
}

/*
 *  Return the contents of memory at the given address. Simulates
 *  instruction fetch. 
 */
unsigned int Fetch ( int addr) {
    return mips.memory[(addr-0x00400000)/4];
}

/* Decode instr, returning decoded instruction. */
void Decode ( unsigned int instr, DecodedInstr* d, RegVals* rVals) {
    /* Your code goes here */
    //Decoding opcode
    //printf("Here is the instr. code: %x \n", instr);
    //instr = Fetch(instr);
    int opcode = ((instr >> 26));
	//printf(" \n Here is the opcode: %x \n", opcode);
	d->op = opcode;

if(d->op == 0x00)
	{
		int rs = ((instr >> 21) & 0x1F);
		rVals->R_rs = rs;
		d->regs.r.rs = rs;
		//printf(" \n Here is the rs: %x \n", rs);

		int rt = ((instr >> 16) & 0x1F);
		rVals->R_rt = rt;
		d->regs.r.rt = rt;
		//printf(" \n Here is the rt: %x \n", rt);

		int rd = ((instr >> 11) & 0x1F);
		rVals->R_rd = rd;
		d->regs.r.rd = rd;
		//printf(" \n Here is the rd: %x \n", rd);
		
		int shift = ((instr >> 6) & 0x1F);
		d->regs.r.shamt = shift;
		//printf(" \n Here is the shamt: %x \n", shift);

		int funct = ((instr >> 0) & 0x3F);
		d->regs.r.funct = funct;
		//printf(" \n Here is the funct: %x \n", funct);
	}
	// I-Format
	else if(d->op == 0x09 || d->op == 0x0C || d->op == 0x04 || d->op == 0x05 || d->op == 0x24 || d->op == 0x25 || d->op == 0x0F || d->op == 0x23 || d->op == 0x0D || d->op == 0x28 || d->op == 0x29 || d->op == 0x0A || d->op == 0x0B || d->op == 0x2B)
	{
		int rs = ((instr >> 21) & 0x1F);
		rVals->R_rs = rs;
		d->regs.i.rs = rs;
//		printf(" \n Here is the rs: %x \n", rs);

		int rt = ((instr >> 16) & 0x1F);
		rVals->R_rt = rt;
		d->regs.i.rt = rt;

	//	printf(" \n Here is the rt: %x \n", rt);
		int IMM = ((instr >> 0) & 0xFFFF);
		signBit = ((IMM>> 15));
		//Do Two's complement if sign Bit is 1
		if(signBit == 1)
		{
			IMM = ~MID(IMM,0,16);
			printf(" \n Here is the IMM: %d \n", IMM);
		}
		rVals->R_rd = IMM;
		d->regs.i.addr_or_immed = IMM;

		printf(" \n Here is the IMM: %d \n", IMM);
	}
	// J-Format
	else if(d->op == 0x02 || d->op == 0x03)
	{
		int target_address = ((instr) & 0x1FFFFFF) * 4;
		d->regs.j.target = target_address;
		//printf(" \n Here is the target_address: %x \n", target_address);
	}
	else
	{
		exit(0);
	}
	
}

/*
 *  Print the disassembled version of the given instruction
 *  followed by a newline.
 */
void PrintInstruction ( DecodedInstr* d) {
    /* Your code goes here */
	//Instruction is R-format
	if(d->op == 0x00)
	{
		//Instruction is addu
		if(d->regs.r.funct == 0x21)
		{
		printf("addu \t $%x, $%x, %x \n", d->regs.r.rd, d->regs.r.rs, d->regs.r.rt);
		}
		//Instruction is subu
		else if(d->regs.r.funct == 0x23)
		{
		printf("subu \t $%d, $%d, %d \n", d->regs.r.rd, d->regs.r.rs, d->regs.r.rt);			
		}
		//Instruction is sll
		else if(d->regs.r.funct == 0x00)
		{
		printf("sll \t $%d, $%d, %d \n", d->regs.r.rd, d->regs.r.rs, d->regs.r.shamt);			
		}
		//Instruction is srl
		else if(d->regs.r.funct == 0x02)
		{
		printf("srl \t $%d, $%d, %d \n", d->regs.r.rd, d->regs.r.rs, d->regs.r.shamt);			
		}
		//Instruction is and
		else if(d->regs.r.funct == 0x24)
		{
		printf("and \t $%d, $%d, %d \n", d->regs.r.rd, d->regs.r.rs, d->regs.r.rt);			
		}
		//Instruction is or
		else if(d->regs.r.funct == 0x25)
		{
		printf("or \t $%d, $%d, %d \n", d->regs.r.rd, d->regs.r.rs, d->regs.r.rt);			
		}
		//Instruction is slt
		else if(d->regs.r.funct == 0x2A)
		{
		printf("slt \t $%d, $%d, %d \n", d->regs.r.rd, d->regs.r.rs, d->regs.r.rt);			
		}
	}
	//Instruction is addiu
	else if(d->op == 0x09)
	{
		if(signBit == 0)
			printf("addiu \t $%d, $%d, %d \n", d->regs.i.rt, d->regs.i.rs , d->regs.i.addr_or_immed);
		else
		{
			printf("addiu \t $%d, $%d, %d \n", d->regs.i.rt, d->regs.i.rs , d->regs.i.addr_or_immed);
		}
	}
	//Instruction is andi
	else if(d->op == 0x0C)
	{
		printf("andi \t $%d, $%d, %d \n", d->regs.i.rt, d->regs.i.rs, d->regs.i.addr_or_immed);

	}
	//Instruction is ori
	else if(d->op == 0x0D)
	{
		printf("ori \t $%d, $%d, %h \n", d->regs.i.rt, d->regs.i.rs, d->regs.i.addr_or_immed);		
	}
	//Instruction is lui
	else if(d->op == 0x0F)
	{
		printf("lui \t $%d, %h \n", d->regs.i.rt, d->regs.i.addr_or_immed);		
	}
	//Instruction is beq
	else if(d->op == 0x04)
	{
		printf("beq \t $%d, $%d, %x \n", d->regs.i.rt, d->regs.i.rs, d->regs.i.addr_or_immed);		
	}
	//Instruction is bne
	else if(d->op == 0x05)
	{
		printf("bne \t $%d, $%d, %x \n", d->regs.i.rt, d->regs.i.rs, d->regs.i.addr_or_immed);		
	}
	//Instruction is J
	else if(d->op == 0x02)
	{
		printf("j \t $%x \n", d->regs.j.target);		
	}
	//Instruction is Jal
	else if(d->op == 0x03)
	{
		printf("jal \t $%x \n", d->regs.j.target);	
	}
	//Instruction is lw
	else if(d->op == 0x23)
	{
		//rs is offset
		printf("lw \t $%d, $%d(%d) \n", d->regs.i.rt, d->regs.i.addr_or_immed, d->regs.i.rs);		
	}
	//Instruction is sw
	else if(d->op == 0x2B)
	{ 
		//rd is offset
		printf("sw \t $%d, $%d(%d) \n", d->regs.i.rt, d->regs.i.addr_or_immed, d->regs.i.rs);		
	}
	
}

/* Perform computation needed to execute d, returning computed value */
int Execute ( DecodedInstr* d, RegVals* rVals) {
    /* Your code goes here */
    unsigned int rs, rt, imm, rd, shamt, jump;
    rs = rt = imm = rd = shamt = jump = 0;
	if(d->op == 0x00)
	{
		//Instruction is addu
		if(d->regs.r.funct == 0x21)
		{
		//mips.register[]
		rs = d->regs.i.rs;
		rt = d->regs.i.rt;
		rd = rs + rt;
		return rd;
		}
		//Instruction is subu
		else if(d->regs.r.funct == 0x23)
		{
		rs = d->regs.i.rs;
		rt = d->regs.i.rt;
		rd = rs - rt;
		return rd;					
		}
		//Instruction is sll
		else if(d->regs.r.funct == 0x00)
		{
		rs = d->regs.i.rs;
		shamt = d->regs.r.shamt;
		rd = rs << shamt;
		return rd;
		}
		//Instruction is srl
		else if(d->regs.r.funct == 0x02)
		{
		rs = d->regs.i.rs;
		shamt = d->regs.r.shamt;
		rd = rs >> shamt;
		return rd;
		}
		//Instruction is and
		else if(d->regs.r.funct == 0x24)
		{
		rs = d->regs.i.rs;
		rt = d->regs.i.rt;
		rd = rs & rt;
		return rd;
		}
		//Instruction is or
		else if(d->regs.r.funct == 0x25)
		{
		rs = d->regs.i.rs;
		rt = d->regs.i.rt;
		rd = rs | rt;
		return rd;
		}
		//Instruction is slt
		else if(d->regs.r.funct == 0x2A)
		{
		rs = d->regs.i.rs;
		rt = d->regs.i.rt;
		rd = (rs < rt);
		}
	}
	//Instruction is addiu
	else if(d->op == 0x09)
	{
		//dereference through *d->i.rt 
		rs = d->regs.i.rs;
		imm = d->regs.i.addr_or_immed;
		rt = rs + imm;
		//printf("Bug: %d", rt);
		return rt;
	}
	//Instruction is andi
	else if(d->op == 0x0C)
	{
		rs = d->regs.i.rs;
		rt = d->regs.i.rt;
		imm = d->regs.i.addr_or_immed;
		rt = rs & imm;
		return rt;
	}
	//Instruction is ori
	else if(d->op == 0x0D)
	{
		rs = d->regs.i.rs;
		rt = d->regs.i.rt;
		imm = d->regs.i.addr_or_immed;
		rt = rs | imm;
		return rt;
	}
	//Instruction is lui
	else if(d->op == 0x0F)
	{
		rs = d->regs.i.rs;
		rt = d->regs.i.rt;
		imm = d->regs.i.addr_or_immed;
		rt = (imm << 16);
		return rt;	
	}
	//Instruction is beq
	else if(d->op == 0x04)
	{
		rs = d->regs.i.rs;
		rt = d->regs.i.rt;
		imm = d->regs.i.addr_or_immed;
		if(rs == rt)
		{
			mips.pc += (imm << 2);
			
		}
		return 0;		
	}
	//Instruction is bne
	else if(d->op == 0x05)
	{
		rs = d->regs.i.rs;
		rt = d->regs.i.rt;
		imm = d->regs.i.addr_or_immed;
		if(rs != rt)
		{
			mips.pc += (imm << 2);
		}
		return 0;
	}
	//Instruction is J
	else if(d->op == 0x02)
	{
		jump = d->regs.j.target;
		mips.pc = jump;		
		return 0;
	}
	//Instruction is Jal
	else if(d->op == 0x03)
	{
		jump = d->regs.j.target;
		imm = mips.pc + 4;
		mips.pc = jump;
		return imm;
	}
	//Instruction is lw
	else if(d->op == 0x23)
	{
		//rs is offset
		rs = d->regs.i.rs;
		return rs;
	}
	//Instruction is sw
	else if(d->op == 0x2B)
	{
		rt = d->regs.i.rt;
		return rt;	
	}
  return 0;
}

/* 
 * Update the program counter based on the current instruction. For
 * instructions other than branches and jumps, for example, the PC
 * increments by 4 (which we have provided).
 */
void UpdatePC ( DecodedInstr* d, int val) {
    mips.pc+=4;
    /* Your code goes here */
}

/*
 * Perform memory load or store. Place the address of any updated memory 
 * in *changedMem, otherwise put -1 in *changedMem. Return any memory value 
 * that is read, otherwise return -1. 
 *
 * Remember that we're mapping MIPS addresses to indices in the mips.memory 
 * array. mips.memory[0] corresponds with address 0x00400000, mips.memory[1] 
 * with address 0x00400004, and so forth.
 *
 */
int Mem( DecodedInstr* d, int val, int *changedMem) {
    /* Your code goes here */
	//Load Word
	   //      printf("Right during Mem: %d \n", val);
	if(((d->regs.i.addr_or_immed * 4) + (int)d->regs.i.rs) >= (int)0x00401000 || (int)0x00403fff >= ((d->regs.i.addr_or_immed * 4) + (int)d->regs.i.rs))
	{	
		if(d->op == 0x23)
		{
			*changedMem = -1; //Not changing memory (Not SW)
			printf("Value (LW): %d \n", val);
			mips.registers[(d->regs.i.rt)] = val;
			return val;
		}
		//Store Word
		else if(d->op == 0x2B)
		{
			*changedMem = (d->regs.i.addr_or_immed * 4) + (int)d->regs.i.rs;
			mips.memory[(d->regs.i.addr_or_immed * 4) + (int)d->regs.i.rs] = val;
			return -1; //Not reading values (Not LW)
		}
		else
		{
			return val;
		}
	}
	else
	{
		printf("Memory Access Exception at 0x%x: address 0x%x \n", mips.pc, ((d->regs.i.addr_or_immed * 4) + (int)d->regs.i.rs));
	}

  return 0;
}

/* 
 * Write back to register. If the instruction modified a register--
 * (including jal, which modifies $ra) --
 * put the index of the modified register in *changedReg,
 * otherwise put -1 in *changedReg.
 */
void RegWrite( DecodedInstr* d, int val, int *changedReg) {
    /* Your code goes here */
	//register[31] is $ra
	if(d->op == 0x03)
	{
		mips.registers[31] = val;
		*changedReg = 31;
	}

	else if(d->op == 0x00)
	{
		*changedReg = d->regs.r.rd;
		mips.registers[d->regs.r.rd] = val;
	}
	else if(d->op == 0x08 || d->op == 0x09 || d->op == 0x0C || d->op == 0x05 || d->op == 0x24 || d->op == 0x25 || d->op == 0x0F || d->op == 0x23 || d->op == 0x0D || d->op == 0x28 || d->op == 0x29 || d->op == 0x0A || d->op == 0x0B || d->op == 0x2B)
	{
		//printf("value: %d\n", val);
		//printf("d->regs.i.rt %d \n", d->regs.i.rt);
		*changedReg = d->regs.i.rt;
		mips.registers[d->regs.i.rt] = val;
	}
	else *changedReg = -1;
}



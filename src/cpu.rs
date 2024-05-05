use std::collections::HashMap;
use crate::opcodes;

bitflags! {
    /// # Status Register (P) http://wiki.nesdev.com/w/index.php/Status_flags
    ///
    ///  7 6 5 4 3 2 1 0
    ///  N V _ B D I Z C
    ///  | |   | | | | +--- Carry Flag
    ///  | |   | | | +----- Zero Flag
    ///  | |   | | +------- Interrupt Disable
    ///  | |   | +--------- Decimal Mode (not used on NES)
    ///  | |   +----------- Break Command
    ///  | +--------------- Overflow Flag
    ///  +----------------- Negative Flag
    ///
    #[derive(Clone)]
    pub struct CpuFlags: u8 {
        const CARRY             = 0b00000001;
        const ZERO              = 0b00000010;
        const INTERRUPT_DISABLE = 0b00000100;
        const DECIMAL_MODE      = 0b00001000;
        const BREAK             = 0b00010000;
        const BREAK2            = 0b00100000;
        const OVERFLOW          = 0b01000000;
        const NEGATIV           = 0b10000000;
    }
}

const STACK: u16 = 0x0100;
const STACK_RESET: u8 = 0xfd;

pub struct CPU {
    pub register_a: u8,
    pub register_x: u8,
    pub register_y: u8,
    pub status: CpuFlags,
    pub program_counter: u16,
    pub stack_pointer: u8,
    memory: [u8; 0xFFFF],
}

#[derive(Debug)]
#[allow(non_camel_case_types)]
pub enum AddressingMode {
   Immediate,
   ZeroPage,
   ZeroPage_X,
   ZeroPage_Y,
   Absolute,
   Absolute_X,
   Absolute_Y,
   Indirect_X,
   Indirect_Y,
   NoneAddressing,
}

trait Mem {
    fn mem_read(&self, addr: u16) -> u8; 

    fn mem_write(&mut self, addr: u16, data: u8);
    
    fn mem_read_u16(&self, pos: u16) -> u16 {
        let lo = self.mem_read(pos) as u16;
        let hi = self.mem_read(pos + 1) as u16;
        (hi << 8) | (lo as u16)
    }

    fn mem_write_u16(&mut self, pos: u16, data: u16) {
        let hi = (data >> 8) as u8;
        let lo = (data & 0xff) as u8;
        self.mem_write(pos, lo);
        self.mem_write(pos + 1, hi);
    }
}

impl Mem for CPU {
    fn mem_read(&self, addr: u16) -> u8 { 
        self.memory[addr as usize]
    }

    fn mem_write(&mut self, addr: u16, data: u8) { 
        self.memory[addr as usize] = data;
    }
}
 
impl CPU {
    pub fn new() -> Self {
        CPU {
            register_a: 0,
            register_x: 0,
            register_y: 0,
            status: CpuFlags::from_bits_truncate(0b100100),
            stack_pointer: STACK_RESET,
            program_counter: 0,
            memory: [0; 0xFFFF],
        }
    }

    fn get_operand_address(&self, mode: &AddressingMode) -> u16 {
        match mode {
            AddressingMode::Immediate => self.program_counter,

            AddressingMode::ZeroPage  => self.mem_read(self.program_counter) as u16,
            
            AddressingMode::Absolute => self.mem_read_u16(self.program_counter),
          
            AddressingMode::ZeroPage_X => {
                let pos = self.mem_read(self.program_counter);
                let addr = pos.wrapping_add(self.register_x) as u16;
                addr
            }
            AddressingMode::ZeroPage_Y => {
                let pos = self.mem_read(self.program_counter);
                let addr = pos.wrapping_add(self.register_y) as u16;
                addr
            }

            AddressingMode::Absolute_X => {
                let base = self.mem_read_u16(self.program_counter);
                let addr = base.wrapping_add(self.register_x as u16);
                addr
            }
            AddressingMode::Absolute_Y => {
                let base = self.mem_read_u16(self.program_counter);
                let addr = base.wrapping_add(self.register_y as u16);
                addr
            }

            AddressingMode::Indirect_X => {
                let base = self.mem_read(self.program_counter);

                let ptr: u8 = (base as u8).wrapping_add(self.register_x);
                let lo = self.mem_read(ptr as u16);
                let hi = self.mem_read(ptr.wrapping_add(1) as u16);
                (hi as u16) << 8 | (lo as u16)
            }
            AddressingMode::Indirect_Y => {
                let base = self.mem_read(self.program_counter);

                let lo = self.mem_read(base as u16);
                let hi = self.mem_read((base as u8).wrapping_add(1) as u16);
                let deref_base = (hi as u16) << 8 | (lo as u16);
                let deref = deref_base.wrapping_add(self.register_y as u16);
                deref
            }
           
            AddressingMode::NoneAddressing => {
                panic!("mode {:?} is not supported", mode);
            }
        }
    }

    //Helper functions
    fn update_zero_flag(&mut self, cond: bool){
        if cond {
            self.status.insert(CpuFlags::ZERO);
        } else {
            self.status.remove(CpuFlags::ZERO);
        }
    }

    fn update_carry_flag(&mut self, cond: bool){
        if cond {
            self.status.insert(CpuFlags::CARRY);
        } else {
            self.status.remove(CpuFlags::CARRY);
        }
    }

    fn update_negative_flag(&mut self, cond: bool){
        if cond {
            self.status.insert(CpuFlags::NEGATIV);
        } else {
            self.status.remove(CpuFlags::NEGATIV);
        }
    }

    fn update_overflow_flag(&mut self, cond: bool){
        if cond {
            self.status.insert(CpuFlags::OVERFLOW);
        } else {
            self.status.remove(CpuFlags::OVERFLOW);
        }
    }

    fn update_zero_and_negative_flags(&mut self, result: u8) {
        self.update_zero_flag(result == 0);
        self.update_negative_flag(result & 0b1000_0000 != 0)
    }

    fn set_register_a(&mut self, value: u8) {
        self.register_a = value;
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn set_register_x(&mut self, value: u8) {
        self.register_x = value;
        self.update_zero_and_negative_flags(self.register_x);
    }

    fn set_register_y(&mut self, value: u8) {
        self.register_y = value;
        self.update_zero_and_negative_flags(self.register_y);
    }

    fn add_to_register_a(&mut self, data: u8) {
        let sum = self.register_a as u16
            + data as u16
            + (if self.status.contains(CpuFlags::CARRY) {
                1
            } else {
                0
            }) as u16;

        let carry = sum > 0xff;

        if carry {
            self.status.insert(CpuFlags::CARRY);
        } else {
            self.status.remove(CpuFlags::CARRY);
        }

        let result = sum as u8;

        if (data ^ result) & (result ^ self.register_a) & 0x80 != 0 {
            self.status.insert(CpuFlags::OVERFLOW);
        } else {
            self.status.remove(CpuFlags::OVERFLOW)
        }

        self.set_register_a(result);
    }

     fn compare(&mut self, result: u8) {
        self.update_carry_flag(result >= 0 as u8);
        self.update_zero_flag(result == 0 as u8);
        self.update_negative_flag((result >> 6) & 1 == 1);
     }

    fn stack_pop(&mut self) -> u8 {
        self.stack_pointer = self.stack_pointer.wrapping_add(1);
        self.mem_read((STACK as u16) + self.stack_pointer as u16)
    }

    fn stack_push(&mut self, data: u8) {
        self.mem_write((STACK as u16) + self.stack_pointer as u16, data);
        self.stack_pointer = self.stack_pointer.wrapping_sub(1)
    }

    fn stack_push_u16(&mut self, data: u16) {
        let hi = (data >> 8) as u8;
        let lo = (data & 0xff) as u8;
        self.stack_push(hi);
        self.stack_push(lo);
    }

    fn stack_pop_u16(&mut self) -> u16 {
        let lo = self.stack_pop() as u16;
        let hi = self.stack_pop() as u16;

        hi << 8 | lo
    }

    // End of Helpers ============

    fn adc(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.add_to_register_a(value);
    }

    fn sbc(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.add_to_register_a((value as i8).wrapping_neg().wrapping_sub(1) as u8);
    }

    fn lda(&mut self, mode: &AddressingMode) {
       let addr = self.get_operand_address(mode);
       let value = self.mem_read(addr);
      
       self.set_register_a(value);
    }

    fn ldx(&mut self, mode: &AddressingMode) {
       let addr = self.get_operand_address(mode);
       let value = self.mem_read(addr);
      
       self.set_register_x(value);
    }

    fn ldy(&mut self, mode: &AddressingMode) {
       let addr = self.get_operand_address(mode);
       let value = self.mem_read(addr);
      
       self.set_register_y(value);
    }

    fn sta(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_a);
    }

    fn stx(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_x);
    }

    fn sty(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_y);
    }

    fn inc(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let mut value = self.mem_read(addr);
        value = value.wrapping_add(1);
        self.mem_write(addr, value);
        self.update_zero_and_negative_flags(value);
    }
  
    fn inx(&mut self) {
        self.register_x = self.register_x.wrapping_add(1);
        self.update_zero_and_negative_flags(self.register_x);
    }
    
    fn iny(&mut self) {
        self.register_y = self.register_y.wrapping_add(1);
        self.update_zero_and_negative_flags(self.register_y);
    }

    fn and(&mut self, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.set_register_a(self.register_a & value);
    }

    fn ora(&mut self, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.set_register_a(self.register_a | value);
    }
    
    fn eor(&mut self, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.set_register_a(self.register_a ^ value);
    }

    fn asl_accumulator(&mut self){
        let mut data = self.register_a;
        self.update_carry_flag(data >> 7 == 1);
        data = data << 1;
        self.set_register_a(data)
    }

    fn asl(&mut self, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        let mut data = self.mem_read(addr);

        self.update_carry_flag(data >> 7 == 1);
        data = data << 1;
        self.mem_write(addr, data);
        self.update_zero_and_negative_flags(data);
        // data
    }

    fn lsr_accumulator(&mut self){
        let mut data = self.register_a;
        self.update_carry_flag(data & 1 == 1);
        data = data >> 1;
        self.set_register_a(data)
    }

    fn lsr(&mut self, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        let mut data = self.mem_read(addr);

        self.update_carry_flag(data & 1 == 1);

        data = data >> 1;
        self.mem_write(addr, data);
        self.update_zero_and_negative_flags(data);
    }

    fn rol_accumulator(&mut self){
        let mut data = self.register_a;
        let old_c = self.status.contains(CpuFlags::CARRY);

        self.update_carry_flag(data >> 7 == 1);

        data = data << 1;
        if old_c {
            data = data | 1;
        }

        self.set_register_a(data);
    }

    fn rol(&mut self, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        let mut data = self.mem_read(addr);
        let old_c = self.status.contains(CpuFlags::CARRY);

        self.update_carry_flag(data >> 7 == 1);

        data = data << 1;
        if old_c {
            data = data | 1;
        }

        self.mem_write(addr, data);
        self.update_zero_and_negative_flags(data);
    }

    fn ror_accumulator(&mut self){
        let mut data = self.register_a;
        let old_carry = self.status.contains(CpuFlags::CARRY);

        self.update_carry_flag(data & 1 == 1);
        data = data >> 1;
        if old_carry {
            data = data | 0b10000000;
        }
        self.set_register_a(data);
    }

    fn ror(&mut self, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        let mut data = self.mem_read(addr);
        let old_carry = self.status.contains(CpuFlags::CARRY);

        self.update_carry_flag(data & 1 == 1);
        data = data >> 1;
        if old_carry {
            data = data | 0b10000000;
        }
        self.mem_write(addr, data);
        self.update_zero_and_negative_flags(data);
    }

    fn php(&mut self) {
        //http://wiki.nesdev.com/w/index.php/CPU_status_flag_behavior
        let mut flags = self.status.clone();
        flags.insert(CpuFlags::BREAK);
        flags.insert(CpuFlags::BREAK2);
        self.stack_push(flags.bits());
    }

    fn plp(&mut self) {
        self.status = CpuFlags::from_bits_truncate(self.stack_pop());
        self.status.remove(CpuFlags::BREAK);
        self.status.insert(CpuFlags::BREAK2);
    }

    fn bit(&mut self, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        let data = self.mem_read(addr);

        self.update_zero_flag(self.register_a & data == 0);
        self.update_overflow_flag((data >> 5) & 1 == 1);
        self.update_negative_flag((data >> 6) & 1 == 1);
    }

    fn cmp(&mut self, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        let result = self.register_a.wrapping_sub(value);

        self.compare(result);
    }

    fn cmp_x(&mut self, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        let result = self.register_x.wrapping_sub(value);

        self.compare(result);
    }

    fn cmp_y(&mut self, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        let result = self.register_y.wrapping_sub(value);

        self.compare(result);
    }

    fn dec(&mut self, mode: &AddressingMode){
        let addr = self.get_operand_address(mode);
        let mut value = self.mem_read(addr);
        value = value.wrapping_sub(1);
        self.mem_write(addr, value);
        self.update_zero_and_negative_flags(value);
    }

    fn dex(&mut self){
        self.register_x = self.register_x.wrapping_sub(1);
        self.update_zero_and_negative_flags(self.register_x);
    }
    
    fn dey(&mut self){
        self.register_y = self.register_y.wrapping_sub(1);
        self.update_zero_and_negative_flags(self.register_y);
    }

    fn jmp_abs(&mut self) {
        let mem_address = self.mem_read_u16(self.program_counter);
        self.program_counter = mem_address;
    }
    
    fn jmp_ind(&mut self) {
        let mem_address = self.mem_read_u16(self.program_counter);
        let indirect_ref = if mem_address & 0x00FF == 0x00FF {
            let lo = self.mem_read(mem_address);
            let hi = self.mem_read(mem_address & 0xFF00);
            (hi as u16) << 8 | (lo as u16)
        } else {
            self.mem_read_u16(mem_address)
        };

        self.program_counter = indirect_ref;
    }

    //CPU loading, running, and initialization
    pub fn load(&mut self, program: Vec<u8>) {
        self.memory[0x8000 .. (0x8000 + program.len())].copy_from_slice(&program[..]);
        self.mem_write_u16(0xFFFC, 0x8000);
    }

    pub fn load_and_run(&mut self, program: Vec<u8>) {
        self.load(program);
        self.reset();
        self.run()
    }

    pub fn reset(&mut self) {
        self.register_a = 0;
        self.register_x = 0;
        self.register_y = 0;
        self.stack_pointer = STACK_RESET;
        self.status = CpuFlags::from_bits_truncate(0b100100);

        self.program_counter = self.mem_read_u16(0xFFFC);
    }

    fn branch(&mut self, condition: bool){
        if condition {
            let jump: i8 = self.mem_read(self.program_counter) as i8;
            let jump_addr = self
                .program_counter
                .wrapping_add(1)
                .wrapping_add(jump as u16);

            self.program_counter = jump_addr;
        }
    }

    pub fn run(&mut self) {
        let ref opcodes: HashMap<u8, &'static opcodes::OpCode> = *opcodes::OPCODES_MAP;

        loop {
            let code = self.mem_read(self.program_counter);
            self.program_counter += 1;
            let program_counter_state = self.program_counter;

            let opcode = opcodes.get(&code).expect(&format!("OpCode {:x} is not recognized", code));

            match code {

                /* ADC - Add with carry */
                0x69 | 0x65 | 0x75 | 0x6D | 0x7D | 0x79 | 0x61 | 0x71 => {
                    self.adc(&opcode.mode);
                }

                /* SBC - Subtract with carry */
                0xE9 | 0xE5 | 0xF5 | 0xED | 0xFD | 0xF9 | 0xE1 | 0xF1 => {
                    self.sbc(&opcode.mode);
                }

                /* LDA - Load Accumulator*/
                0xa9 | 0xa5 | 0xb5 | 0xad | 0xbd | 0xb9 | 0xa1 | 0xb1 => {
                    self.lda(&opcode.mode);
                }

                /* LDX - Load X Register */
                0xa2 | 0xa6 | 0xb6 | 0xae | 0xbe => {
                    self.ldx(&opcode.mode);
                }

                /* LDY - Load Y Register */
                0xa0 | 0xa4 | 0xb4 | 0xac | 0xbc => {
                    self.ldy(&opcode.mode);
                }

                /* STA - Store Accumulator */
                0x85 | 0x95 | 0x8d | 0x9d | 0x99 | 0x81 | 0x91 => {
                    self.sta(&opcode.mode);
                }

                /* STX - Store X Register */
                0x86 | 0x96 | 0x8e => {
                    self.stx(&opcode.mode);
                }

                /* STY - Store Y Register */
                0x84 | 0x94 | 0x8c => {
                    self.sty(&opcode.mode);
                }

                /* TAX - Transfer Accumulator to X */
                0xaa => self.set_register_x(self.register_a),

                /* TAY - Transfer Accumulator to Y */
                0xa8 => self.set_register_y(self.register_a),

                /* TSX - Transfer Stack Pointer to X */
                0xba => self.set_register_x(self.stack_pointer),
                
                /* TXA - Transfer X to Accumulator */
                0x8a => self.set_register_a(self.register_x),

                /* TXS - Transfer X to Stack Pointer */
                0x9a => self.stack_pointer = self.register_x,

                /* TYA - Transfer Y to Accumulator */
                0x98 => self.set_register_a(self.register_y),
                
                /* INC - Increment Memory */
                0xe6 | 0xf6 | 0xee | 0xfe => {
                    self.inc(&opcode.mode);
                }
                /* INX - Increment X Register */
                0xe8 => self.inx(),
                /* INY - Increment Y Register */
                0xc8 => self.iny(),

                /* AND - Logical And */
                0x29 | 0x25 | 0x35 | 0x2d | 0x3d | 0x39 | 0x21 | 0x31 => {
                    self.and(&opcode.mode);
                }

                /* ASL - Arithmetic Shift Left */
                0x0a => self.asl_accumulator(), 

                0x06 | 0x16 | 0x0e | 0x1e => {
                    self.asl(&opcode.mode);
                }
                
                /* LSR - Logical Shift Right */
                0x4a => self.lsr_accumulator(),

                0x46 | 0x56 | 0x4e | 0x5e => {
                    self.lsr(&opcode.mode);
                }

                /* ROL - Rotate Left */
                0x2a => self.rol_accumulator(),

                0x26 | 0x36 | 0x2e | 0x3e => {
                    self.rol(&opcode.mode);
                }

                /* ROR - Rotate Right*/
                0x6a => self.ror_accumulator(),

                0x66 | 0x76 | 0x6e | 0x7e => {
                    self.ror(&opcode.mode);
                }
                
                /* BCC - Branch if Carry Clear */
                0x90 => self.branch(!self.status.contains(CpuFlags::CARRY)),
                /* BCS - Branch if Carry Set */
                0xb0 => self.branch(self.status.contains(CpuFlags::CARRY)),
                
                /* BEQ - Branch if Equal */
                0xf0 => self.branch(self.status.contains(CpuFlags::ZERO)),
                /* BNE - Branch if Not Equal */
                0xd0 => self.branch(!self.status.contains(CpuFlags::ZERO)),

                /* BMI - Branch if Minus */
                0x30 => self.branch(self.status.contains(CpuFlags::NEGATIV)),
                /* BPL - Branch if Positive */
                0x10 => self.branch(!self.status.contains(CpuFlags::NEGATIV)),

                /* BVC - Branch if Overflow Clear */
                0x50 => self.branch(!self.status.contains(CpuFlags::OVERFLOW)),
                /* BVS - Branch if Overflow Set */
                0x70 => self.branch(self.status.contains(CpuFlags::OVERFLOW)),

                /* BIT - Bit Test */
                0x24 | 0x2c => self.bit(&opcode.mode),

                /* CMP - Compare */
                0xc9 | 0xc5 | 0xd5 | 0xcd | 0xdd | 0xd9 | 0xc1 | 0xd1 => {
                    self.cmp(&opcode.mode);
                }
                /* CPX - Compare X Register */
                0xe0 | 0xe4 | 0xec => {
                    self.cmp_x(&opcode.mode);
                }
                /* CPY - Compare Y Register */
                0xc0 | 0xc4 | 0xcc => {
                    self.cmp_y(&opcode.mode);
                }

                /* DEC - Decrement Memory */
                0xc6 | 0xd6 | 0xce | 0xde => {
                    self.dec(&opcode.mode);
                }
                /* DEX - Decrement X Register */
                0xca => self.dex(),
                /* DEY - Decrement Y Register */
                0x88 => self.dey(),


                /* CLD - Clear Decimal Mode */
                0xd8 => self.status.remove(CpuFlags::DECIMAL_MODE),
                /* CLC - Clear Carry Flag */
                0x18 => self.status.remove(CpuFlags::CARRY),
                /* CLI - Clear Interrupt Disable */
                0x58 => self.status.remove(CpuFlags::INTERRUPT_DISABLE),
                /* CLV - Clear Overflow Flag */
                0xb8 => self.status.remove(CpuFlags::OVERFLOW),

                /* ORA - Logical Inclusive OR */
                0x09 | 0x05 | 0x15 | 0x0d | 0x1d | 0x19 | 0x01 | 0x11 => {
                    self.ora(&opcode.mode);
                }

                /* EOR - Exclusive OR */
                0x49 | 0x45 | 0x55 | 0x4d | 0x5d | 0x59 | 0x41 | 0x51 => {
                    self.eor(&opcode.mode);
                }

                /* PHA - Push Accumulator */
                0x48 => self.stack_push(self.register_a),

                /* PHP - Push Processor Status */
                0x08 => self.php(),

                /* PLP - Pull Processor Status */
                0x28 => self.plp(),

                /* PLA - Pull Accumulator */
                0x68 => {
                    let data = self.stack_pop();
                    self.set_register_a(data);
                }

                /* JMP - Jump */
                0x4c => self.jmp_abs(),
                0x6c => self.jmp_ind(),

                /* JSR - Jump to Subroutine */
                0x20 => {
                    self.stack_push_u16(self.program_counter + 2 - 1);
                    let target_address = self.mem_read_u16(self.program_counter);
                    self.program_counter = target_address
                } 

                /* RTS - Return from Subroutine */
                0x60 => {
                    self.program_counter = self.stack_pop_u16();
                }

                /* RTI - Return from Interrupt */
                0x40 => {
                    self.status = CpuFlags::from_bits_truncate(self.stack_pop());
                    self.status.remove(CpuFlags::BREAK);
                    self.status.insert(CpuFlags::BREAK2);

                    self.program_counter = self.stack_pop_u16();
                }

                /* SEC- Set Carry Flag */
                0x38 => self.status.insert(CpuFlags::CARRY),

                /* SED- Set Decimal Flag */
                0xf8 => self.status.insert(CpuFlags::DECIMAL_MODE),

                /* SEI- Set Interrupt Disable */
                0x78 => self.status.insert(CpuFlags::INTERRUPT_DISABLE),

                /* NOP - No Operation */
                0xea => {}

                /* BRK - Break */
                0x00 => return,
                _ => todo!(),
            }//End Match

            if program_counter_state == self.program_counter {
                self.program_counter += (opcode.len - 1) as u16;
            }

        }//End loop
    }//End run
}//End CPU

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_0xa9_lda_immediate_load_data() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x05, 0x00]);
        assert_eq!(cpu.register_a, 5);
        assert!(cpu.status & 0b0000_0010 == 0);
        assert!(cpu.status & 0b1000_0000 == 0);
    }

    #[test]
    fn test_0xa9_lda_zero_flag() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x00, 0x00]);
        assert!(cpu.status & 0b0000_0010 == 0b10);
    }

    #[test]
    fn test_0xaa_tax_move_a_to_x() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x0A,0xaa, 0x00]);

        assert_eq!(cpu.register_x, 10)
    }

    #[test]
    fn test_5_ops_working_together() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0xc0, 0xaa, 0xe8, 0x00]);

        assert_eq!(cpu.register_x, 0xc1)
    }

    #[test]
    fn test_inx_overflow() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0xff, 0xaa,0xe8, 0xe8, 0x00]);

        assert_eq!(cpu.register_x, 1)
    }

    #[test]
    fn test_lda_from_memory() {
        let mut cpu = CPU::new();
        cpu.mem_write(0x10, 0x55);

        cpu.load_and_run(vec![0xa5, 0x10, 0x00]);

        assert_eq!(cpu.register_a, 0x55);
    }
}
